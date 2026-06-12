use super::arrangement3d::{ExactArrangement, ExactTopologyAssemblyReport};
use super::boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanEvaluation, ExactBooleanRequest,
    ExactIdenticalMeshReport, arrangement_boolean_attempt_report_from_arrangement,
    evaluate_boolean_exact_request_with_artifacts_and_arrangement_replay,
    materialize_certified_boolean_support_with_arrangement,
    materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph,
    materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph,
    validate_boolean_result_against_sources_with_artifacts,
};
use super::cell_complex::{
    ExactRegionOwnershipReport, ExactSelectedCellComplex, ExactSelectedCellComplexFreshness,
    select_arrangement_for_replay,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{
    ExactIntersectionGraph, IntersectionGraphValidationError, build_intersection_graph,
};
use super::mesh::ExactMesh;
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::reports::{
    ExactAdjacentUnionCompletionReport, ExactBooleanPreflight, ExactBooleanResult,
    ExactBoundaryTouchingReport, ExactOpenSurfaceDisjointReport, ExactPlanarArrangementReport,
    ExactRefinementReport, ExactReportFreshness, ExactReportValidationError,
    ExactSameSurfaceReport, ExactVolumetricBoundaryClosureReport, ExactWindingReadinessReport,
};
use super::simplify::{ExactSimplifiedCellComplex, ExactSimplifiedCellComplexFreshness};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceError, CoplanarVolumetricCellEvidenceFreshness,
    CoplanarVolumetricCellEvidenceReport,
};

/// Reusable exact boolean session for a fixed source-mesh pair.
///
/// The workspace keeps source meshes borrowed and caches replayable exact
/// artifacts. It does not weaken freshness: every artifact is still built from
/// the same retained source objects and can be validated against those sources.
#[derive(Debug)]
pub struct ExactBooleanWorkspace<'a> {
    left: &'a ExactMesh,
    right: &'a ExactMesh,
    graph: Option<ExactIntersectionGraph>,
    coplanar_volumetric_cell_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
    closed_boundary_touching_regularized_materializations: Vec<(
        ExactBooleanRequest,
        Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>,
    )>,
    closed_no_volume_overlap_regularized_materializations: Vec<(
        ExactBooleanRequest,
        Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>,
    )>,
    arrangements: Vec<(ExactRegularizationPolicy, ExactArrangement)>,
    topology_assembly_reports: Vec<(ExactRegularizationPolicy, ExactTopologyAssemblyReport)>,
    region_ownership_reports: Vec<(ExactRegularizationPolicy, ExactRegionOwnershipReport)>,
    arrangement_attempts: Vec<(
        ExactBooleanRequest,
        ExactRegularizationPolicy,
        ExactArrangementBooleanAttempt,
    )>,
    selected_cell_complexes: Vec<(
        ExactBooleanRequest,
        ExactRegularizationPolicy,
        ExactSelectedCellComplex,
    )>,
    simplified_cell_complexes: Vec<(
        ExactBooleanRequest,
        ExactRegularizationPolicy,
        ExactSimplifiedCellComplex,
    )>,
    refinement_reports: Vec<(ExactBooleanRequest, ExactRefinementReport)>,
    adjacent_union_completion_reports:
        Vec<(ExactBooleanRequest, ExactAdjacentUnionCompletionReport)>,
    identical_mesh_reports: Vec<(ExactBooleanRequest, ExactIdenticalMeshReport)>,
    same_surface_reports: Vec<(ExactBooleanRequest, ExactSameSurfaceReport)>,
    boundary_touching_reports: Vec<(ExactBooleanRequest, ExactBoundaryTouchingReport)>,
    open_surface_disjoint_reports: Vec<(ExactBooleanRequest, ExactOpenSurfaceDisjointReport)>,
    volumetric_boundary_closure_reports:
        Vec<(ExactBooleanRequest, ExactVolumetricBoundaryClosureReport)>,
    preflights: Vec<(ExactBooleanRequest, ExactBooleanPreflight)>,
    winding_readiness_reports: Vec<(ExactBooleanRequest, ExactWindingReadinessReport)>,
    planar_arrangement_reports: Vec<(ExactBooleanRequest, ExactPlanarArrangementReport)>,
    evaluations: Vec<(ExactBooleanRequest, ExactBooleanEvaluation)>,
    materializations: Vec<(ExactBooleanRequest, ExactBooleanResult)>,
}

impl<'a> ExactBooleanWorkspace<'a> {
    /// Creates an empty exact workspace for a fixed left/right mesh pair.
    pub fn new(left: &'a ExactMesh, right: &'a ExactMesh) -> Self {
        Self {
            left,
            right,
            graph: None,
            coplanar_volumetric_cell_evidence: None,
            closed_boundary_touching_regularized_materializations: Vec::new(),
            closed_no_volume_overlap_regularized_materializations: Vec::new(),
            arrangements: Vec::new(),
            topology_assembly_reports: Vec::new(),
            region_ownership_reports: Vec::new(),
            arrangement_attempts: Vec::new(),
            selected_cell_complexes: Vec::new(),
            simplified_cell_complexes: Vec::new(),
            refinement_reports: Vec::new(),
            adjacent_union_completion_reports: Vec::new(),
            identical_mesh_reports: Vec::new(),
            same_surface_reports: Vec::new(),
            boundary_touching_reports: Vec::new(),
            open_surface_disjoint_reports: Vec::new(),
            volumetric_boundary_closure_reports: Vec::new(),
            preflights: Vec::new(),
            winding_readiness_reports: Vec::new(),
            planar_arrangement_reports: Vec::new(),
            evaluations: Vec::new(),
            materializations: Vec::new(),
        }
    }

    /// Returns the left source mesh.
    pub fn left(&self) -> &'a ExactMesh {
        self.left
    }

    /// Returns the right source mesh.
    pub fn right(&self) -> &'a ExactMesh {
        self.right
    }

    /// Returns the exact intersection graph, building it once per workspace.
    pub fn graph(&mut self) -> Result<&ExactIntersectionGraph, MeshError> {
        if self.graph.is_none() {
            self.graph = Some(build_intersection_graph(self.left, self.right)?);
        }
        Ok(self
            .graph
            .as_ref()
            .expect("intersection graph was just initialized"))
    }

    /// Returns retained coplanar volumetric-cell evidence, deriving it from
    /// the workspace's cached exact intersection graph.
    pub fn coplanar_volumetric_cell_evidence(
        &mut self,
    ) -> Result<&CoplanarVolumetricCellEvidenceReport, MeshError> {
        if self.coplanar_volumetric_cell_evidence.is_none() {
            self.graph()?;
            let graph = self
                .graph
                .as_ref()
                .expect("intersection graph cache was just populated");
            graph
                .validate_against_meshes(self.left, self.right)
                .map_err(workspace_graph_validation_error)?;
            let report =
                CoplanarVolumetricCellEvidenceReport::from_graph(graph, self.left, self.right);
            report
                .validate()
                .map_err(workspace_coplanar_volumetric_cell_error)?;
            self.coplanar_volumetric_cell_evidence = Some(report);
        }
        Ok(self
            .coplanar_volumetric_cell_evidence
            .as_ref()
            .expect("coplanar volumetric-cell evidence cache was just populated"))
    }

    /// Validate coplanar volumetric-cell evidence against this workspace's
    /// retained source session.
    pub fn validate_coplanar_volumetric_cell_evidence(
        &mut self,
        report: &CoplanarVolumetricCellEvidenceReport,
    ) -> Result<(), CoplanarVolumetricCellEvidenceError> {
        if self
            .coplanar_volumetric_cell_evidence
            .as_ref()
            .is_some_and(|stored_report| stored_report == report)
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify coplanar volumetric-cell evidence freshness in this retained
    /// source session.
    pub fn coplanar_volumetric_cell_evidence_freshness(
        &mut self,
        report: &CoplanarVolumetricCellEvidenceReport,
    ) -> CoplanarVolumetricCellEvidenceFreshness {
        match self.validate_coplanar_volumetric_cell_evidence(report) {
            Ok(()) => CoplanarVolumetricCellEvidenceFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Materializes zero-area closed boundary contact from the retained exact
    /// graph, caching both certified output and consumed coplanar evidence.
    pub fn materialize_closed_boundary_touching_regularized_with_evidence(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
        if let Some((_, cached)) = self
            .closed_boundary_touching_regularized_materializations
            .iter()
            .find(|(stored_request, _)| *stored_request == request)
        {
            validate_cached_result_with_evidence(cached)?;
            return Ok(cached.clone());
        }

        self.graph()?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        graph
            .validate_against_meshes(self.left, self.right)
            .map_err(workspace_graph_validation_error)?;
        let materialized =
            materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                graph,
                self.left,
                self.right,
                request.operation,
                request.validation,
            )?;
        validate_cached_result_with_evidence(&materialized)?;
        self.closed_boundary_touching_regularized_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes positive-area closed boundary contact with no shared
    /// volume from the retained exact graph, caching output and evidence.
    pub fn materialize_closed_no_volume_overlap_regularized_with_evidence(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
        if let Some((_, cached)) = self
            .closed_no_volume_overlap_regularized_materializations
            .iter()
            .find(|(stored_request, _)| *stored_request == request)
        {
            validate_cached_result_with_evidence(cached)?;
            return Ok(cached.clone());
        }

        self.graph()?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        graph
            .validate_against_meshes(self.left, self.right)
            .map_err(workspace_graph_validation_error)?;
        let materialized =
            materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                graph,
                self.left,
                self.right,
                request.operation,
                request.validation,
            )?;
        validate_cached_result_with_evidence(&materialized)?;
        self.closed_no_volume_overlap_regularized_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Returns the exact arrangement for `policy`, building it once per policy.
    pub fn arrangement(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangement, MeshError> {
        if let Some(index) = self
            .arrangements
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
        {
            return Ok(&self.arrangements[index].1);
        }

        let arrangement = ExactArrangement::from_meshes_with_policy(self.left, self.right, policy)?;
        self.arrangements.push((policy, arrangement));
        Ok(&self
            .arrangements
            .last()
            .expect("arrangement cache was just populated")
            .1)
    }

    /// Returns topology-assembly evidence for `policy`, reusing the cached
    /// arrangement and report for that policy.
    pub fn topology_assembly_report(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactTopologyAssemblyReport, MeshError> {
        if let Some(index) = self
            .topology_assembly_reports
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
        {
            return Ok(&self.topology_assembly_reports[index].1);
        }

        self.arrangement(policy)?;
        let arrangement_index = self
            .arrangements
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
            .expect("arrangement cache was just populated");
        let report = self.arrangements[arrangement_index]
            .1
            .topology_assembly_report_with_policy(self.left, self.right, policy);
        self.topology_assembly_reports.push((policy, report));
        Ok(&self
            .topology_assembly_reports
            .last()
            .expect("topology assembly report cache was just populated")
            .1)
    }

    /// Validate topology-assembly evidence against this workspace's retained
    /// source session.
    pub fn validate_topology_assembly_report(
        &mut self,
        policy: ExactRegularizationPolicy,
        report: &ExactTopologyAssemblyReport,
    ) -> Result<(), ExactArrangementBlocker> {
        if self
            .topology_assembly_reports
            .iter()
            .any(|(stored_policy, stored_report)| {
                *stored_policy == policy && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        report.validate_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Returns region-ownership evidence for `policy`, reusing the cached
    /// arrangement and report for that policy.
    pub fn region_ownership_report(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactRegionOwnershipReport, MeshError> {
        if let Some(index) = self
            .region_ownership_reports
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
        {
            return Ok(&self.region_ownership_reports[index].1);
        }

        self.arrangement(policy)?;
        let arrangement_index = self
            .arrangements
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
            .expect("arrangement cache was just populated");
        let report = self.arrangements[arrangement_index]
            .1
            .region_ownership_report_with_policy(self.left, self.right, policy)
            .map_err(workspace_arrangement_blocker_error)?;
        self.region_ownership_reports.push((policy, report));
        Ok(&self
            .region_ownership_reports
            .last()
            .expect("region ownership report cache was just populated")
            .1)
    }

    /// Validate region-ownership evidence against this workspace's retained
    /// source session.
    pub fn validate_region_ownership_report(
        &mut self,
        policy: ExactRegularizationPolicy,
        report: &ExactRegionOwnershipReport,
    ) -> Result<(), ExactArrangementBlocker> {
        if self
            .region_ownership_reports
            .iter()
            .any(|(stored_policy, stored_report)| {
                *stored_policy == policy && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        report.validate_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Returns the arrangement/cell-complex attempt report for `request` and
    /// `policy`, reusing the cached arrangement for that policy.
    pub fn arrangement_attempt(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangementBooleanAttempt, MeshError> {
        if let Some(index) =
            self.arrangement_attempts
                .iter()
                .position(|(stored_request, stored_policy, _)| {
                    *stored_request == request && *stored_policy == policy
                })
        {
            return Ok(&self.arrangement_attempts[index].2);
        }

        self.arrangement(policy)?;
        let arrangement_index = self
            .arrangements
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
            .expect("arrangement cache was just populated");
        let attempt = arrangement_boolean_attempt_report_from_arrangement(
            self.left,
            self.right,
            request,
            policy,
            &self.arrangements[arrangement_index].1,
        )?;
        self.arrangement_attempts.push((request, policy, attempt));
        Ok(&self
            .arrangement_attempts
            .last()
            .expect("arrangement attempt cache was just populated")
            .2)
    }

    /// Validate arrangement/cell-complex attempt evidence against this
    /// workspace's retained source session.
    pub fn validate_arrangement_attempt(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        attempt: &ExactArrangementBooleanAttempt,
    ) -> Result<(), ExactReportValidationError> {
        if self.arrangement_attempts.iter().any(
            |(stored_request, stored_policy, stored_attempt)| {
                *stored_request == request && *stored_policy == policy && stored_attempt == attempt
            },
        ) {
            attempt.validate()?;
            return Ok(());
        }

        self.arrangement(policy)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        let arrangement_index = self
            .arrangements
            .iter()
            .position(|(stored_policy, _)| *stored_policy == policy)
            .expect("arrangement cache was just populated");
        attempt.validate()?;
        let replay = arrangement_boolean_attempt_report_from_arrangement(
            self.left,
            self.right,
            request,
            policy,
            &self.arrangements[arrangement_index].1,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        replay.validate()?;
        if attempt == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify arrangement/cell-complex attempt freshness in this retained
    /// source session.
    pub fn arrangement_attempt_freshness(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        attempt: &ExactArrangementBooleanAttempt,
    ) -> ExactReportFreshness {
        match self.validate_arrangement_attempt(request, policy, attempt) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns selected exact cell-complex evidence for `request` and `policy`,
    /// retaining the topology and ownership reports consumed by selection.
    pub fn selected_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactSelectedCellComplex, MeshError> {
        if let Some(index) =
            self.selected_cell_complexes
                .iter()
                .position(|(stored_request, stored_policy, _)| {
                    *stored_request == request && *stored_policy == policy
                })
        {
            return Ok(&self.selected_cell_complexes[index].2);
        }

        let arrangement = self.arrangement(policy)?.clone();
        let selected = select_arrangement_for_replay(
            arrangement,
            self.left,
            self.right,
            request.operation,
            policy,
        )
        .map_err(workspace_arrangement_blocker_error)?;
        self.selected_cell_complexes
            .push((request, policy, selected));
        Ok(&self
            .selected_cell_complexes
            .last()
            .expect("selected cell-complex cache was just populated")
            .2)
    }

    /// Returns simplified exact cell-complex evidence for `request` and
    /// `policy`, retaining the gate reports consumed before simplification.
    pub fn simplified_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactSimplifiedCellComplex, MeshError> {
        if let Some(index) =
            self.simplified_cell_complexes
                .iter()
                .position(|(stored_request, stored_policy, _)| {
                    *stored_request == request && *stored_policy == policy
                })
        {
            return Ok(&self.simplified_cell_complexes[index].2);
        }

        let selected = self.selected_cell_complex(request, policy)?.clone();
        let simplified = selected
            .simplify_exact_with_policy(policy)
            .map_err(workspace_arrangement_blocker_error)?;
        self.simplified_cell_complexes
            .push((request, policy, simplified));
        Ok(&self
            .simplified_cell_complexes
            .last()
            .expect("simplified cell-complex cache was just populated")
            .2)
    }

    /// Validate selected cell-complex evidence against this workspace's
    /// retained source session.
    pub fn validate_selected_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        selected: &ExactSelectedCellComplex,
    ) -> Result<(), ExactArrangementBlocker> {
        if self.selected_cell_complexes.iter().any(
            |(stored_request, stored_policy, stored_selected)| {
                *stored_request == request
                    && *stored_policy == policy
                    && stored_selected == selected
            },
        ) {
            selected.validate()?;
            return Ok(());
        }
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        selected.validate_against_arrangement(arrangement, self.left, self.right, policy)
    }

    /// Classify selected cell-complex freshness in this retained source
    /// session.
    pub fn selected_cell_complex_freshness(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        selected: &ExactSelectedCellComplex,
    ) -> ExactSelectedCellComplexFreshness {
        match self.validate_selected_cell_complex(request, policy, selected) {
            Ok(()) => ExactSelectedCellComplexFreshness::Current,
            Err(_) => ExactSelectedCellComplexFreshness::StaleSelectedCells,
        }
    }

    /// Validate simplified cell-complex evidence against this workspace's
    /// retained source session.
    pub fn validate_simplified_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        simplified: &ExactSimplifiedCellComplex,
    ) -> Result<(), ExactArrangementBlocker> {
        if self.simplified_cell_complexes.iter().any(
            |(stored_request, stored_policy, stored_simplified)| {
                *stored_request == request
                    && *stored_policy == policy
                    && stored_simplified == simplified
            },
        ) {
            simplified.validate()?;
            return Ok(());
        }
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        simplified.validate_against_arrangement(arrangement, self.left, self.right, policy)
    }

    /// Classify simplified cell-complex freshness in this retained source
    /// session.
    pub fn simplified_cell_complex_freshness(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        simplified: &ExactSimplifiedCellComplex,
    ) -> ExactSimplifiedCellComplexFreshness {
        match self.validate_simplified_cell_complex(request, policy, simplified) {
            Ok(()) => ExactSimplifiedCellComplexFreshness::Current,
            Err(_) => ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells,
        }
    }

    /// Returns preflight for `request`, building it once per request.
    pub fn preflight(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanPreflight, MeshError> {
        if let Some(index) = self
            .preflights
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.preflights[index].1);
        }

        let preflight = request.preflight(self.left, self.right)?;
        self.preflights.push((request, preflight));
        Ok(&self
            .preflights
            .last()
            .expect("preflight cache was just populated")
            .1)
    }

    /// Validate preflight scheduling evidence against this workspace's source
    /// meshes.
    pub fn validate_preflight(
        &mut self,
        request: ExactBooleanRequest,
        preflight: &ExactBooleanPreflight,
    ) -> Result<(), ExactReportValidationError> {
        if preflight.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .preflights
            .iter()
            .any(|(stored_request, stored_preflight)| {
                *stored_request == request && stored_preflight == preflight
            })
        {
            preflight.validate()?;
            return Ok(());
        }
        preflight.validate_against_sources_with_boundary_policy(
            self.left,
            self.right,
            request.validation,
            request.boundary_policy,
        )
    }

    /// Classify preflight freshness in this retained source session.
    pub fn preflight_freshness(
        &mut self,
        request: ExactBooleanRequest,
        preflight: &ExactBooleanPreflight,
    ) -> ExactReportFreshness {
        match self.validate_preflight(request, preflight) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns refinement evidence for `request`, building it once per
    /// request.
    pub fn refinement_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactRefinementReport, MeshError> {
        if let Some(index) = self
            .refinement_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.refinement_reports[index].1);
        }

        let report = request.refinement_report(self.left, self.right)?;
        self.refinement_reports.push((request, report));
        Ok(&self
            .refinement_reports
            .last()
            .expect("refinement report cache was just populated")
            .1)
    }

    /// Validate refinement evidence against this workspace's source meshes.
    pub fn validate_refinement_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactRefinementReport,
    ) -> Result<(), ExactReportValidationError> {
        if report.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .refinement_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify refinement-report freshness in this retained source session.
    pub fn refinement_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactRefinementReport,
    ) -> ExactReportFreshness {
        match self.validate_refinement_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns adjacent-union completion evidence for `request`, building it
    /// once per request.
    pub fn adjacent_union_completion_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactAdjacentUnionCompletionReport, MeshError> {
        if let Some(index) = self
            .adjacent_union_completion_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.adjacent_union_completion_reports[index].1);
        }

        let report = request.adjacent_union_completion_report(self.left, self.right)?;
        self.adjacent_union_completion_reports
            .push((request, report));
        Ok(&self
            .adjacent_union_completion_reports
            .last()
            .expect("adjacent-union completion report cache was just populated")
            .1)
    }

    /// Validate adjacent-union completion evidence against this workspace's
    /// source meshes.
    pub fn validate_adjacent_union_completion_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactAdjacentUnionCompletionReport,
    ) -> Result<(), ExactReportValidationError> {
        if report.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .adjacent_union_completion_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify adjacent-union completion freshness in this retained source
    /// session.
    pub fn adjacent_union_completion_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactAdjacentUnionCompletionReport,
    ) -> ExactReportFreshness {
        match self.validate_adjacent_union_completion_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns identical-mesh evidence for `request`, building it once per
    /// request.
    pub fn identical_mesh_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> &ExactIdenticalMeshReport {
        if let Some(index) = self
            .identical_mesh_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return &self.identical_mesh_reports[index].1;
        }

        let report = request.identical_mesh_report(self.left, self.right);
        self.identical_mesh_reports.push((request, report));
        &self
            .identical_mesh_reports
            .last()
            .expect("identical-mesh report cache was just populated")
            .1
    }

    /// Validate identical-mesh evidence against this workspace's source
    /// meshes.
    pub fn validate_identical_mesh_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactIdenticalMeshReport,
    ) -> Result<(), ExactReportValidationError> {
        if self
            .identical_mesh_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify identical-mesh freshness in this retained source session.
    pub fn identical_mesh_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactIdenticalMeshReport,
    ) -> ExactReportFreshness {
        match self.validate_identical_mesh_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns same-surface evidence for `request`, building it once per
    /// request.
    pub fn same_surface_report(&mut self, request: ExactBooleanRequest) -> &ExactSameSurfaceReport {
        if let Some(index) = self
            .same_surface_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return &self.same_surface_reports[index].1;
        }

        let report = request.same_surface_report(self.left, self.right);
        self.same_surface_reports.push((request, report));
        &self
            .same_surface_reports
            .last()
            .expect("same-surface report cache was just populated")
            .1
    }

    /// Validate same-surface evidence against this workspace's source meshes.
    pub fn validate_same_surface_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactSameSurfaceReport,
    ) -> Result<(), ExactReportValidationError> {
        if self
            .same_surface_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify same-surface freshness in this retained source session.
    pub fn same_surface_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactSameSurfaceReport,
    ) -> ExactReportFreshness {
        match self.validate_same_surface_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns boundary-touching evidence for `request`, building it once per
    /// request.
    pub fn boundary_touching_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBoundaryTouchingReport, MeshError> {
        if let Some(index) = self
            .boundary_touching_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.boundary_touching_reports[index].1);
        }

        let report = request.boundary_touching_report(self.left, self.right)?;
        self.boundary_touching_reports.push((request, report));
        Ok(&self
            .boundary_touching_reports
            .last()
            .expect("boundary-touching report cache was just populated")
            .1)
    }

    /// Validate boundary-touching evidence against this workspace's source
    /// meshes.
    pub fn validate_boundary_touching_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactBoundaryTouchingReport,
    ) -> Result<(), ExactReportValidationError> {
        if self
            .boundary_touching_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify boundary-touching freshness in this retained source session.
    pub fn boundary_touching_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactBoundaryTouchingReport,
    ) -> ExactReportFreshness {
        match self.validate_boundary_touching_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns open-surface disjointness evidence for `request`, building it
    /// once per request.
    pub fn open_surface_disjoint_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactOpenSurfaceDisjointReport, MeshError> {
        if let Some(index) = self
            .open_surface_disjoint_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.open_surface_disjoint_reports[index].1);
        }

        let report = request.open_surface_disjoint_report(self.left, self.right)?;
        self.open_surface_disjoint_reports.push((request, report));
        Ok(&self
            .open_surface_disjoint_reports
            .last()
            .expect("open-surface disjoint report cache was just populated")
            .1)
    }

    /// Validate open-surface disjointness evidence against this workspace's
    /// source meshes.
    pub fn validate_open_surface_disjoint_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactOpenSurfaceDisjointReport,
    ) -> Result<(), ExactReportValidationError> {
        if self
            .open_surface_disjoint_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify open-surface disjointness freshness in this retained source
    /// session.
    pub fn open_surface_disjoint_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactOpenSurfaceDisjointReport,
    ) -> ExactReportFreshness {
        match self.validate_open_surface_disjoint_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns volumetric boundary-closure evidence for `request`, building it
    /// once per request.
    pub fn volumetric_boundary_closure(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactVolumetricBoundaryClosureReport, MeshError> {
        if let Some(index) = self
            .volumetric_boundary_closure_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.volumetric_boundary_closure_reports[index].1);
        }

        let report = request.volumetric_boundary_closure(self.left, self.right)?;
        self.volumetric_boundary_closure_reports
            .push((request, report));
        Ok(&self
            .volumetric_boundary_closure_reports
            .last()
            .expect("volumetric boundary-closure report cache was just populated")
            .1)
    }

    /// Validate volumetric boundary-closure evidence against this workspace's
    /// source meshes.
    pub fn validate_volumetric_boundary_closure(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactVolumetricBoundaryClosureReport,
    ) -> Result<(), ExactReportValidationError> {
        if report.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .volumetric_boundary_closure_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify volumetric boundary-closure freshness in this retained source
    /// session.
    pub fn volumetric_boundary_closure_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactVolumetricBoundaryClosureReport,
    ) -> ExactReportFreshness {
        match self.validate_volumetric_boundary_closure(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns winding-readiness evidence for `request`, building it once per
    /// request.
    pub fn winding_readiness(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactWindingReadinessReport, MeshError> {
        if let Some(index) = self
            .winding_readiness_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.winding_readiness_reports[index].1);
        }

        let readiness = request.winding_readiness(self.left, self.right)?;
        self.winding_readiness_reports.push((request, readiness));
        Ok(&self
            .winding_readiness_reports
            .last()
            .expect("winding-readiness cache was just populated")
            .1)
    }

    /// Validate winding-readiness evidence against this workspace's source
    /// meshes.
    pub fn validate_winding_readiness(
        &mut self,
        request: ExactBooleanRequest,
        readiness: &ExactWindingReadinessReport,
    ) -> Result<(), ExactReportValidationError> {
        if readiness.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .winding_readiness_reports
            .iter()
            .any(|(stored_request, stored_readiness)| {
                *stored_request == request && stored_readiness == readiness
            })
        {
            readiness.validate()?;
            return Ok(());
        }
        readiness.validate_against_sources_with_boundary_policy(
            self.left,
            self.right,
            request.validation,
            request.boundary_policy,
        )
    }

    /// Classify winding-readiness freshness in this retained source session.
    pub fn winding_readiness_freshness(
        &mut self,
        request: ExactBooleanRequest,
        readiness: &ExactWindingReadinessReport,
    ) -> ExactReportFreshness {
        match self.validate_winding_readiness(request, readiness) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns planar-arrangement readiness evidence for `request`, building
    /// it once per request.
    pub fn planar_arrangement_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactPlanarArrangementReport, MeshError> {
        if let Some(index) = self
            .planar_arrangement_reports
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.planar_arrangement_reports[index].1);
        }

        let report = request.planar_arrangement_report(self.left, self.right)?;
        self.planar_arrangement_reports.push((request, report));
        Ok(&self
            .planar_arrangement_reports
            .last()
            .expect("planar-arrangement report cache was just populated")
            .1)
    }

    /// Validate planar-arrangement readiness evidence against this workspace's
    /// source meshes.
    pub fn validate_planar_arrangement_report(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactPlanarArrangementReport,
    ) -> Result<(), ExactReportValidationError> {
        if report.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self
            .planar_arrangement_reports
            .iter()
            .any(|(stored_request, stored_report)| {
                *stored_request == request && stored_report == report
            })
        {
            report.validate()?;
            return Ok(());
        }
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify planar-arrangement readiness freshness in this retained source
    /// session.
    pub fn planar_arrangement_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactPlanarArrangementReport,
    ) -> ExactReportFreshness {
        match self.validate_planar_arrangement_report(request, report) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Returns an exact boolean evaluation for `request`, building it once per
    /// request.
    pub fn evaluate(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanEvaluation, MeshError> {
        if let Some(index) = self
            .evaluations
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
        {
            return Ok(&self.evaluations[index].1);
        }

        self.preflight(request)?;
        self.graph()?;
        if !matches!(
            request.operation,
            super::boolean::ExactBooleanOperation::SelectedRegions(_)
        ) {
            self.arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)?;
        }
        let preflight_index = self
            .preflights
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
            .expect("preflight cache was just populated");
        let preflight = &self.preflights[preflight_index].1;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self
            .arrangements
            .iter()
            .find(|(stored_policy, _)| {
                *stored_policy == ExactRegularizationPolicy::REGULARIZED_SOLID
            })
            .map(|(_, arrangement)| arrangement);
        let evaluation = evaluate_boolean_exact_request_with_artifacts_and_arrangement_replay(
            self.left,
            self.right,
            request,
            preflight,
            graph,
            regularized_arrangement,
            false,
        )?;
        self.evaluations.push((request, evaluation));
        Ok(&self
            .evaluations
            .last()
            .expect("evaluation cache was just populated")
            .1)
    }

    /// Materializes `request`, reusing a cached certified evaluation when the
    /// workspace has one and otherwise reusing retained graph, preflight, and
    /// arrangement artifacts for certified supports.
    pub fn materialize(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanResult, MeshError> {
        if let Some((_, result)) = self
            .materializations
            .iter()
            .find(|(stored_request, _)| *stored_request == request)
        {
            result
                .validate()
                .map_err(workspace_report_validation_error)?;
            return Ok(result.clone());
        }
        if let Some(result) = self
            .evaluations
            .iter()
            .find(|(stored_request, _)| *stored_request == request)
            .and_then(|(_, evaluation)| evaluation.result.as_ref())
        {
            result
                .validate()
                .map_err(workspace_report_validation_error)?;
            return Ok(result.clone());
        }
        self.preflight(request)?;
        self.graph()?;
        if !matches!(
            request.operation,
            super::boolean::ExactBooleanOperation::SelectedRegions(_)
        ) {
            self.arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)?;
        }
        let preflight_index = self
            .preflights
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
            .expect("preflight cache was just populated");
        let preflight = &self.preflights[preflight_index].1;
        if preflight.is_certified() {
            let regularized_arrangement = self
                .arrangements
                .iter()
                .find(|(stored_policy, _)| {
                    *stored_policy == ExactRegularizationPolicy::REGULARIZED_SOLID
                })
                .map(|(_, arrangement)| arrangement);
            let result = materialize_certified_boolean_support_with_arrangement(
                self.left,
                self.right,
                request,
                preflight.support,
                regularized_arrangement,
            )?;
            self.materializations.push((request, result.clone()));
            return Ok(result);
        }
        let result = request.materialize(self.left, self.right)?;
        self.materializations.push((request, result.clone()));
        Ok(result)
    }

    /// Validate an evaluation against this workspace's source meshes using
    /// retained graph and arrangement artifacts where they apply.
    pub fn validate_evaluation(
        &mut self,
        evaluation: &ExactBooleanEvaluation,
    ) -> Result<(), ExactReportValidationError> {
        self.graph()
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if !matches!(
            evaluation.request.operation,
            super::boolean::ExactBooleanOperation::SelectedRegions(_)
        ) {
            self.arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self
            .arrangements
            .iter()
            .find(|(stored_policy, _)| {
                *stored_policy == ExactRegularizationPolicy::REGULARIZED_SOLID
            })
            .map(|(_, arrangement)| arrangement);
        evaluation.validate_against_sources_with_artifacts(
            self.left,
            self.right,
            graph,
            regularized_arrangement,
        )
    }

    /// Classify an evaluation's freshness against this workspace's source
    /// meshes using the retained replay session.
    pub fn evaluation_freshness(
        &mut self,
        evaluation: &ExactBooleanEvaluation,
    ) -> ExactReportFreshness {
        match self.validate_evaluation(evaluation) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }

    /// Validate a materialized result against this workspace's source meshes
    /// using retained graph artifacts where they apply.
    pub fn validate_result(
        &mut self,
        request: ExactBooleanRequest,
        result: &ExactBooleanResult,
    ) -> Result<(), ExactReportValidationError> {
        if self
            .materializations
            .iter()
            .any(|(stored_request, stored_result)| {
                *stored_request == request && stored_result == result
            })
            || self
                .evaluations
                .iter()
                .filter(|(stored_request, _)| *stored_request == request)
                .filter_map(|(_, evaluation)| evaluation.result.as_ref())
                .any(|stored_result| stored_result == result)
        {
            result.validate()?;
            return Ok(());
        }

        self.graph()
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if !matches!(
            request.operation,
            super::boolean::ExactBooleanOperation::SelectedRegions(_)
        ) {
            self.arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self
            .arrangements
            .iter()
            .find(|(stored_policy, _)| {
                *stored_policy == ExactRegularizationPolicy::REGULARIZED_SOLID
            })
            .map(|(_, arrangement)| arrangement);
        validate_boolean_result_against_sources_with_artifacts(
            result,
            graph,
            regularized_arrangement,
            self.left,
            self.right,
            request.operation,
            request.validation,
            request.boundary_policy,
        )
    }

    /// Classify a materialized result's freshness against this workspace's
    /// source meshes using the retained replay session.
    pub fn result_freshness(
        &mut self,
        request: ExactBooleanRequest,
        result: &ExactBooleanResult,
    ) -> ExactReportFreshness {
        match self.validate_result(request, result) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

fn workspace_arrangement_blocker_error(blocker: ExactArrangementBlocker) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact boolean workspace arrangement report failed: {blocker:?}"),
    ))
}

fn workspace_graph_validation_error(error: IntersectionGraphValidationError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact boolean workspace graph failed validation: {error:?}"),
    ))
}

fn workspace_coplanar_volumetric_cell_error(
    error: CoplanarVolumetricCellEvidenceError,
) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!(
            "exact boolean workspace coplanar volumetric evidence failed validation: {error:?}"
        ),
    ))
}

fn validate_cached_result_with_evidence(
    materialized: &Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>,
) -> Result<(), MeshError> {
    if let Some((result, evidence)) = materialized {
        result
            .validate()
            .map_err(workspace_report_validation_error)?;
        evidence
            .validate()
            .map_err(workspace_coplanar_volumetric_cell_error)?;
    }
    Ok(())
}

fn workspace_report_validation_error(error: ExactReportValidationError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact boolean workspace cached report failed validation: {error:?}"),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::ExactBooleanOperation;
    use crate::validation::ValidationPolicy;
    use crate::{
        CoplanarVolumetricCellEvidenceError, CoplanarVolumetricCellEvidenceFreshness,
        ExactBooleanResultKind, ExactReportValidationError, Triangle,
        certify_coplanar_volumetric_cell_evidence,
    };

    #[test]
    fn exact_boolean_workspace_reuses_graph_arrangement_preflight_and_evaluation() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        assert_eq!(workspace.left(), &left);
        assert_eq!(workspace.right(), &right);

        let first_graph = workspace.graph().unwrap() as *const ExactIntersectionGraph;
        let second_graph = workspace.graph().unwrap() as *const ExactIntersectionGraph;
        assert_eq!(first_graph, second_graph);
        workspace
            .graph()
            .unwrap()
            .validate_against_meshes(&left, &right)
            .unwrap();

        let first_coplanar_volumetric_evidence =
            workspace.coplanar_volumetric_cell_evidence().unwrap()
                as *const CoplanarVolumetricCellEvidenceReport;
        let second_coplanar_volumetric_evidence =
            workspace.coplanar_volumetric_cell_evidence().unwrap()
                as *const CoplanarVolumetricCellEvidenceReport;
        assert_eq!(
            first_coplanar_volumetric_evidence,
            second_coplanar_volumetric_evidence
        );
        assert_eq!(
            workspace.coplanar_volumetric_cell_evidence().unwrap(),
            &certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap()
        );
        let coplanar_volumetric_evidence = workspace
            .coplanar_volumetric_cell_evidence()
            .unwrap()
            .clone();
        workspace
            .validate_coplanar_volumetric_cell_evidence(&coplanar_volumetric_evidence)
            .unwrap();
        assert_eq!(
            workspace.coplanar_volumetric_cell_evidence_freshness(&coplanar_volumetric_evidence),
            CoplanarVolumetricCellEvidenceFreshness::Current
        );
        let mut stale_coplanar_volumetric_evidence = coplanar_volumetric_evidence.clone();
        stale_coplanar_volumetric_evidence.retained_face_pair_count += 1;
        assert_eq!(
            workspace
                .validate_coplanar_volumetric_cell_evidence(&stale_coplanar_volumetric_evidence),
            Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch)
        );
        assert_eq!(
            workspace
                .coplanar_volumetric_cell_evidence_freshness(&stale_coplanar_volumetric_evidence),
            CoplanarVolumetricCellEvidenceFreshness::StaleFacePairCounts
        );

        let first_arrangement = workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangement;
        let second_arrangement = workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangement;
        assert_eq!(first_arrangement, second_arrangement);

        let first_topology_report = workspace
            .topology_assembly_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactTopologyAssemblyReport;
        let second_topology_report = workspace
            .topology_assembly_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactTopologyAssemblyReport;
        assert_eq!(first_topology_report, second_topology_report);
        workspace
            .topology_assembly_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .validate()
            .unwrap();
        let topology_report = workspace
            .topology_assembly_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        topology_report
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace
            .validate_topology_assembly_report(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &topology_report,
            )
            .unwrap();
        let mut stale_topology_report = topology_report.clone();
        stale_topology_report.graph_events += 1;
        stale_topology_report.validate().unwrap();
        assert_eq!(
            stale_topology_report.validate_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
        assert_eq!(
            workspace.validate_topology_assembly_report(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_topology_report,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let first_ownership_report = workspace
            .region_ownership_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactRegionOwnershipReport;
        let second_ownership_report = workspace
            .region_ownership_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactRegionOwnershipReport;
        assert_eq!(first_ownership_report, second_ownership_report);
        workspace
            .region_ownership_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .validate()
            .unwrap();
        let ownership_report = workspace
            .region_ownership_report(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        ownership_report
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace
            .validate_region_ownership_report(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &ownership_report,
            )
            .unwrap();
        let mut stale_ownership_report = ownership_report.clone();
        stale_ownership_report.face_cell_boundary_nodes += 3;
        stale_ownership_report.face_cell_boundary_points += 3;
        stale_ownership_report.validate().unwrap();
        assert_eq!(
            stale_ownership_report.validate_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
        assert_eq!(
            workspace.validate_region_ownership_report(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_ownership_report,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let first_attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangementBooleanAttempt;
        let second_attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangementBooleanAttempt;
        assert_eq!(first_attempt, second_attempt);
        assert_eq!(
            workspace
                .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap(),
            &request
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID,)
                .unwrap()
        );
        let attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        attempt.validate().unwrap();
        attempt.validate_against_sources(&left, &right).unwrap();
        workspace
            .validate_arrangement_attempt(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &attempt,
            )
            .unwrap();
        assert_eq!(
            workspace.arrangement_attempt_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &attempt,
            ),
            ExactReportFreshness::Current
        );
        let mut stale_attempt = attempt.clone();
        stale_attempt
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        stale_attempt.validate().unwrap();
        assert!(
            workspace
                .validate_arrangement_attempt(
                    request,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                    &stale_attempt,
                )
                .is_err()
        );
        assert_ne!(
            workspace.arrangement_attempt_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_attempt,
            ),
            ExactReportFreshness::Current
        );

        let first_selected = workspace
            .selected_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactSelectedCellComplex;
        let second_selected = workspace
            .selected_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactSelectedCellComplex;
        assert_eq!(first_selected, second_selected);
        let selected = workspace
            .selected_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        assert!(selected.topology_assembly_report.is_some());
        assert!(selected.region_ownership_report.is_some());
        selected.validate().unwrap();
        selected
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace
            .validate_selected_cell_complex(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &selected,
            )
            .unwrap();
        assert_eq!(
            workspace.selected_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &selected,
            ),
            ExactSelectedCellComplexFreshness::Current
        );
        let mut stale_selected = selected.clone();
        stale_selected
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        assert!(
            workspace
                .validate_selected_cell_complex(
                    request,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                    &stale_selected,
                )
                .is_err()
        );
        assert_ne!(
            workspace.selected_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_selected,
            ),
            ExactSelectedCellComplexFreshness::Current
        );

        let first_simplified = workspace
            .simplified_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactSimplifiedCellComplex;
        let second_simplified = workspace
            .simplified_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactSimplifiedCellComplex;
        assert_eq!(first_simplified, second_simplified);
        let simplified = workspace
            .simplified_cell_complex(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        assert!(simplified.topology_assembly_report.is_some());
        assert!(simplified.region_ownership_report.is_some());
        simplified.validate().unwrap();
        simplified
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace
            .validate_simplified_cell_complex(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &simplified,
            )
            .unwrap();
        assert_eq!(
            workspace.simplified_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &simplified,
            ),
            ExactSimplifiedCellComplexFreshness::Current
        );
        let mut stale_simplified = simplified.clone();
        stale_simplified
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        assert!(
            workspace
                .validate_simplified_cell_complex(
                    request,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                    &stale_simplified,
                )
                .is_err()
        );
        assert_ne!(
            workspace.simplified_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_simplified,
            ),
            ExactSimplifiedCellComplexFreshness::Current
        );

        let first_preflight = workspace.preflight(request).unwrap() as *const ExactBooleanPreflight;
        let second_preflight =
            workspace.preflight(request).unwrap() as *const ExactBooleanPreflight;
        assert_eq!(first_preflight, second_preflight);
        assert_eq!(
            workspace.preflight(request).unwrap(),
            &request.preflight(&left, &right).unwrap()
        );
        let preflight = workspace.preflight(request).unwrap().clone();
        workspace.validate_preflight(request, &preflight).unwrap();
        assert_eq!(
            workspace.preflight_freshness(request, &preflight),
            ExactReportFreshness::Current
        );
        let mut stale_preflight = preflight.clone();
        stale_preflight.retained_events += 1;
        assert_eq!(
            workspace.validate_preflight(request, &stale_preflight),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.preflight_freshness(request, &stale_preflight),
            ExactReportFreshness::Current
        );
        let mut relabeled_preflight = preflight.clone();
        relabeled_preflight.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace.validate_preflight(request, &relabeled_preflight),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let first_refinement_report =
            workspace.refinement_report(request).unwrap() as *const ExactRefinementReport;
        let second_refinement_report =
            workspace.refinement_report(request).unwrap() as *const ExactRefinementReport;
        assert_eq!(first_refinement_report, second_refinement_report);
        assert_eq!(
            workspace.refinement_report(request).unwrap(),
            &request.refinement_report(&left, &right).unwrap()
        );
        let refinement_report = workspace.refinement_report(request).unwrap().clone();
        workspace
            .validate_refinement_report(request, &refinement_report)
            .unwrap();
        assert_eq!(
            workspace.refinement_report_freshness(request, &refinement_report),
            ExactReportFreshness::Current
        );
        let mut stale_refinement_report = refinement_report.clone();
        stale_refinement_report.retained_events += 1;
        assert_eq!(
            workspace.validate_refinement_report(request, &stale_refinement_report),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.refinement_report_freshness(request, &stale_refinement_report),
            ExactReportFreshness::Current
        );
        let mut relabeled_refinement_report = refinement_report.clone();
        relabeled_refinement_report.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace.validate_refinement_report(request, &relabeled_refinement_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let first_adjacent_report = workspace.adjacent_union_completion_report(request).unwrap()
            as *const ExactAdjacentUnionCompletionReport;
        let second_adjacent_report = workspace.adjacent_union_completion_report(request).unwrap()
            as *const ExactAdjacentUnionCompletionReport;
        assert_eq!(first_adjacent_report, second_adjacent_report);
        assert_eq!(
            workspace.adjacent_union_completion_report(request).unwrap(),
            &request
                .adjacent_union_completion_report(&left, &right)
                .unwrap()
        );
        let adjacent_report = workspace
            .adjacent_union_completion_report(request)
            .unwrap()
            .clone();
        workspace
            .validate_adjacent_union_completion_report(request, &adjacent_report)
            .unwrap();
        assert_eq!(
            workspace.adjacent_union_completion_report_freshness(request, &adjacent_report),
            ExactReportFreshness::Current
        );
        let mut stale_adjacent_report = adjacent_report.clone();
        stale_adjacent_report.stronger_kernel_available =
            !stale_adjacent_report.stronger_kernel_available;
        assert_eq!(
            workspace.validate_adjacent_union_completion_report(request, &stale_adjacent_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            workspace.adjacent_union_completion_report_freshness(request, &stale_adjacent_report),
            ExactReportFreshness::Current
        );
        let mut relabeled_adjacent_report = adjacent_report.clone();
        relabeled_adjacent_report.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace
                .validate_adjacent_union_completion_report(request, &relabeled_adjacent_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let first_identical_report =
            workspace.identical_mesh_report(request) as *const ExactIdenticalMeshReport;
        let second_identical_report =
            workspace.identical_mesh_report(request) as *const ExactIdenticalMeshReport;
        assert_eq!(first_identical_report, second_identical_report);
        assert_eq!(
            workspace.identical_mesh_report(request),
            &request.identical_mesh_report(&left, &right)
        );
        let identical_report = workspace.identical_mesh_report(request).clone();
        workspace
            .validate_identical_mesh_report(request, &identical_report)
            .unwrap();
        assert_eq!(
            workspace.identical_mesh_report_freshness(request, &identical_report),
            ExactReportFreshness::Current
        );
        let mut stale_identical_report = identical_report.clone();
        stale_identical_report.left_triangles += 1;
        assert_eq!(
            workspace.validate_identical_mesh_report(request, &stale_identical_report),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.identical_mesh_report_freshness(request, &stale_identical_report),
            ExactReportFreshness::Current
        );

        let first_same_surface_report =
            workspace.same_surface_report(request) as *const ExactSameSurfaceReport;
        let second_same_surface_report =
            workspace.same_surface_report(request) as *const ExactSameSurfaceReport;
        assert_eq!(first_same_surface_report, second_same_surface_report);
        assert_eq!(
            workspace.same_surface_report(request),
            &request.same_surface_report(&left, &right)
        );
        let same_surface_report = workspace.same_surface_report(request).clone();
        workspace
            .validate_same_surface_report(request, &same_surface_report)
            .unwrap();
        assert_eq!(
            workspace.same_surface_report_freshness(request, &same_surface_report),
            ExactReportFreshness::Current
        );
        let mut stale_same_surface_report = same_surface_report.clone();
        stale_same_surface_report.predicates.clear();
        assert_eq!(
            workspace.validate_same_surface_report(request, &stale_same_surface_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            workspace.same_surface_report_freshness(request, &stale_same_surface_report),
            ExactReportFreshness::Current
        );

        let first_boundary_report = workspace.boundary_touching_report(request).unwrap()
            as *const ExactBoundaryTouchingReport;
        let second_boundary_report = workspace.boundary_touching_report(request).unwrap()
            as *const ExactBoundaryTouchingReport;
        assert_eq!(first_boundary_report, second_boundary_report);
        assert_eq!(
            workspace.boundary_touching_report(request).unwrap(),
            &request.boundary_touching_report(&left, &right).unwrap()
        );
        let boundary_report = workspace.boundary_touching_report(request).unwrap().clone();
        workspace
            .validate_boundary_touching_report(request, &boundary_report)
            .unwrap();
        assert_eq!(
            workspace.boundary_touching_report_freshness(request, &boundary_report),
            ExactReportFreshness::Current
        );
        let mut stale_boundary_report = boundary_report.clone();
        stale_boundary_report.retained_events += 1;
        assert_eq!(
            workspace.validate_boundary_touching_report(request, &stale_boundary_report),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.boundary_touching_report_freshness(request, &stale_boundary_report),
            ExactReportFreshness::Current
        );

        let first_open_surface_report = workspace.open_surface_disjoint_report(request).unwrap()
            as *const ExactOpenSurfaceDisjointReport;
        let second_open_surface_report = workspace.open_surface_disjoint_report(request).unwrap()
            as *const ExactOpenSurfaceDisjointReport;
        assert_eq!(first_open_surface_report, second_open_surface_report);
        assert_eq!(
            workspace.open_surface_disjoint_report(request).unwrap(),
            &request.open_surface_disjoint_report(&left, &right).unwrap()
        );
        let open_surface_report = workspace
            .open_surface_disjoint_report(request)
            .unwrap()
            .clone();
        workspace
            .validate_open_surface_disjoint_report(request, &open_surface_report)
            .unwrap();
        assert_eq!(
            workspace.open_surface_disjoint_report_freshness(request, &open_surface_report),
            ExactReportFreshness::Current
        );
        let mut stale_open_surface_report = open_surface_report.clone();
        stale_open_surface_report.left_open_surface = !stale_open_surface_report.left_open_surface;
        assert_eq!(
            workspace.validate_open_surface_disjoint_report(request, &stale_open_surface_report),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.open_surface_disjoint_report_freshness(request, &stale_open_surface_report),
            ExactReportFreshness::Current
        );

        let first_closure_report = workspace.volumetric_boundary_closure(request).unwrap()
            as *const ExactVolumetricBoundaryClosureReport;
        let second_closure_report = workspace.volumetric_boundary_closure(request).unwrap()
            as *const ExactVolumetricBoundaryClosureReport;
        assert_eq!(first_closure_report, second_closure_report);
        assert_eq!(
            workspace.volumetric_boundary_closure(request).unwrap(),
            &request.volumetric_boundary_closure(&left, &right).unwrap()
        );
        let closure_report = workspace
            .volumetric_boundary_closure(request)
            .unwrap()
            .clone();
        workspace
            .validate_volumetric_boundary_closure(request, &closure_report)
            .unwrap();
        assert_eq!(
            workspace.volumetric_boundary_closure_freshness(request, &closure_report),
            ExactReportFreshness::Current
        );
        let mut stale_closure_report = closure_report.clone();
        stale_closure_report.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace.validate_volumetric_boundary_closure(request, &stale_closure_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            workspace.volumetric_boundary_closure_freshness(request, &stale_closure_report),
            ExactReportFreshness::Current
        );

        let first_readiness =
            workspace.winding_readiness(request).unwrap() as *const ExactWindingReadinessReport;
        let second_readiness =
            workspace.winding_readiness(request).unwrap() as *const ExactWindingReadinessReport;
        assert_eq!(first_readiness, second_readiness);
        assert_eq!(
            workspace.winding_readiness(request).unwrap(),
            &request.winding_readiness(&left, &right).unwrap()
        );
        let readiness = workspace.winding_readiness(request).unwrap().clone();
        workspace
            .validate_winding_readiness(request, &readiness)
            .unwrap();
        assert_eq!(
            workspace.winding_readiness_freshness(request, &readiness),
            ExactReportFreshness::Current
        );
        let mut stale_readiness = readiness.clone();
        stale_readiness.retained_events += 1;
        assert_eq!(
            workspace.validate_winding_readiness(request, &stale_readiness),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.winding_readiness_freshness(request, &stale_readiness),
            ExactReportFreshness::Current
        );
        let mut relabeled_readiness = readiness.clone();
        relabeled_readiness.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace.validate_winding_readiness(request, &relabeled_readiness),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let first_planar_report = workspace.planar_arrangement_report(request).unwrap()
            as *const ExactPlanarArrangementReport;
        let second_planar_report = workspace.planar_arrangement_report(request).unwrap()
            as *const ExactPlanarArrangementReport;
        assert_eq!(first_planar_report, second_planar_report);
        assert_eq!(
            workspace.planar_arrangement_report(request).unwrap(),
            &request.planar_arrangement_report(&left, &right).unwrap()
        );
        let planar_report = workspace
            .planar_arrangement_report(request)
            .unwrap()
            .clone();
        workspace
            .validate_planar_arrangement_report(request, &planar_report)
            .unwrap();
        assert_eq!(
            workspace.planar_arrangement_report_freshness(request, &planar_report),
            ExactReportFreshness::Current
        );
        let mut stale_planar_report = planar_report.clone();
        stale_planar_report.retained_events += 1;
        assert_eq!(
            workspace.validate_planar_arrangement_report(request, &stale_planar_report),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            workspace.planar_arrangement_report_freshness(request, &stale_planar_report),
            ExactReportFreshness::Current
        );
        let mut relabeled_planar_report = planar_report.clone();
        relabeled_planar_report.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            workspace.validate_planar_arrangement_report(request, &relabeled_planar_report),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut materialize_workspace = ExactBooleanWorkspace::new(&left, &right);
        materialize_workspace.graph().unwrap();
        materialize_workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        materialize_workspace.preflight(request).unwrap();
        let materialized = materialize_workspace.materialize(request).unwrap();
        assert_eq!(materialized, request.materialize(&left, &right).unwrap());
        assert!(
            materialize_workspace.evaluations.is_empty(),
            "first-call materialize should not populate the evaluation cache"
        );
        assert_eq!(materialize_workspace.materializations.len(), 1);
        assert_eq!(
            materialize_workspace.materialize(request).unwrap(),
            materialized
        );
        assert_eq!(
            materialize_workspace.materializations.len(),
            1,
            "repeated materialize should reuse the cached result"
        );
        let mut corrupt_materialization_cache = ExactBooleanWorkspace::new(&left, &right);
        corrupt_materialization_cache.materialize(request).unwrap();
        corrupt_materialization_cache.materializations[0]
            .1
            .graph_had_unknowns = !corrupt_materialization_cache.materializations[0]
            .1
            .graph_had_unknowns;
        assert!(
            corrupt_materialization_cache.materialize(request).is_err(),
            "cached materialization results must validate before reuse"
        );
        materialize_workspace
            .validate_result(request, &materialized)
            .unwrap();
        let mut locally_invalid_cached_result = materialized.clone();
        locally_invalid_cached_result.graph_had_unknowns =
            !locally_invalid_cached_result.graph_had_unknowns;
        assert!(
            materialize_workspace
                .validate_result(request, &locally_invalid_cached_result)
                .is_err()
        );
        if materialized.topology_assembly_report.is_some() {
            let mut stale_gate_report = materialized.clone();
            stale_gate_report
                .topology_assembly_report
                .as_mut()
                .unwrap()
                .graph_events += 1;
            assert_eq!(
                materialize_workspace.validate_result(request, &stale_gate_report),
                Err(ExactReportValidationError::SourceReplayMismatch)
            );
        }
        assert_eq!(
            materialize_workspace.result_freshness(request, &materialized),
            ExactReportFreshness::Current
        );
        let mut stale_result = materialized.clone();
        stale_result.kind = ExactBooleanResultKind::ArrangementCellComplexMaterialized {
            operation: ExactBooleanOperation::Difference,
        };
        assert!(
            materialize_workspace
                .validate_result(request, &stale_result)
                .is_err()
        );
        assert_ne!(
            materialize_workspace.result_freshness(request, &stale_result),
            ExactReportFreshness::Current
        );

        let first_evaluation =
            workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        let second_evaluation =
            workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(first_evaluation, second_evaluation);
        workspace.evaluate(request).unwrap().validate().unwrap();
        let retained_evaluation = workspace.evaluate(request).unwrap().clone();
        workspace.validate_evaluation(&retained_evaluation).unwrap();
        assert_eq!(
            workspace.evaluation_freshness(&retained_evaluation),
            ExactReportFreshness::Current
        );
        let mut stale_evaluation = retained_evaluation.clone();
        stale_evaluation.preflight.retained_events += 1;
        assert_eq!(
            workspace.validate_evaluation(&stale_evaluation),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        workspace
            .evaluate(request)
            .unwrap()
            .validate_against_sources(&left, &right)
            .unwrap();
        let mut corrupt_evaluation_cache = ExactBooleanWorkspace::new(&left, &right);
        corrupt_evaluation_cache.evaluate(request).unwrap();
        let cached_result = corrupt_evaluation_cache.evaluations[0]
            .1
            .result
            .as_mut()
            .expect("certified test request should retain a result");
        cached_result.graph_had_unknowns = !cached_result.graph_had_unknowns;
        assert!(
            corrupt_evaluation_cache.materialize(request).is_err(),
            "cached evaluation results must validate before materialization reuse"
        );
        assert_eq!(
            workspace.evaluate(request).unwrap(),
            &request.evaluate(&left, &right).unwrap()
        );

        assert_eq!(
            workspace.materialize(request).unwrap(),
            request.materialize(&left, &right).unwrap()
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_boundary_touching_regularized_materialization() {
        let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
        let left = combine_exact_meshes(
            &left_a,
            &left_b,
            "workspace disconnected closed boundary fixture",
        );
        let right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_closed_boundary_touching_regularized_with_evidence(request)
            .unwrap()
            .expect("closed boundary contact should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_closed_boundary_touching_regularized_with_evidence(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(
            workspace
                .closed_boundary_touching_regularized_materializations
                .len(),
            1
        );
        assert_eq!(
            workspace
                .materialize_closed_boundary_touching_regularized_with_evidence(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(
            workspace
                .closed_boundary_touching_regularized_materializations
                .len(),
            1
        );

        let cached_result = &mut workspace.closed_boundary_touching_regularized_materializations[0]
            .1
            .as_mut()
            .unwrap()
            .0;
        cached_result.graph_had_unknowns = !cached_result.graph_had_unknowns;
        assert!(
            workspace
                .materialize_closed_boundary_touching_regularized_with_evidence(request)
                .is_err(),
            "cached boundary-touching materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_no_volume_overlap_materialization() {
        let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
        let left = combine_exact_meshes(
            &left_a,
            &left_b,
            "workspace disconnected positive-area boundary fixture",
        );
        let right = tetra_from_corners([2, 0, 0], [6, 0, 0], [2, 4, 0], [2, 0, -4]);
        let request = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        );
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_closed_no_volume_overlap_regularized_with_evidence(request)
            .unwrap()
            .expect("positive-area boundary contact should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_closed_no_volume_overlap_regularized_with_evidence(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(
            workspace
                .closed_no_volume_overlap_regularized_materializations
                .len(),
            1
        );
        assert_eq!(
            workspace
                .materialize_closed_no_volume_overlap_regularized_with_evidence(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(
            workspace
                .closed_no_volume_overlap_regularized_materializations
                .len(),
            1
        );

        workspace.closed_no_volume_overlap_regularized_materializations[0]
            .1
            .as_mut()
            .unwrap()
            .1
            .retained_face_pair_count += 1;
        assert!(
            workspace
                .materialize_closed_no_volume_overlap_regularized_with_evidence(request)
                .is_err(),
            "cached no-volume materialization evidence must validate before reuse"
        );
    }

    fn tetra(offset: [i64; 3]) -> ExactMesh {
        let [ox, oy, oz] = offset;
        ExactMesh::from_i64_triangles(
            &[ox, oy, oz, ox + 1, oy, oz, ox, oy + 1, oz, ox, oy, oz + 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn tetra_from_corners(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn combine_exact_meshes(left: &ExactMesh, right: &ExactMesh, label: &'static str) -> ExactMesh {
        let right_offset = left.vertices().len();
        ExactMesh::new(
            left.vertices()
                .iter()
                .chain(right.vertices())
                .cloned()
                .collect(),
            left.triangles()
                .iter()
                .copied()
                .chain(right.triangles().iter().map(|triangle| {
                    let [a, b, c] = triangle.0;
                    Triangle([a + right_offset, b + right_offset, c + right_offset])
                }))
                .collect(),
            hyperlimit::SourceProvenance::exact(label),
        )
        .unwrap()
    }
}
