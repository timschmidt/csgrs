use super::arrangement3d::{
    ExactArrangement, ExactTopologyAssemblyReport, ExactTopologyAssemblyStatus,
};
use super::boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanCertificationSet, ExactBooleanEvaluation,
    ExactBooleanRequest, ExactIdenticalMeshReport,
    adjacent_union_completion_certification_from_graph,
    arrangement_boolean_attempt_report_from_arrangement,
    boolean_closed_validation_regularized_meshes, boundary_touching_report_from_graph,
    direct_arrangement_cell_complex_attempt,
    materialize_adjacent_union_completion_from_graph_for_request,
    materialize_boundary_touching_policy_from_graph_for_request,
    materialize_certified_boolean_support_with_artifacts,
    materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph,
    materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph,
    materialize_closed_winding_containment_from_graph_for_request,
    materialize_closed_winding_separated_from_graph_for_request,
    materialize_open_surface_disjoint_from_graph_for_request,
    open_surface_disjoint_report_from_graph, planar_arrangement_report_from_graph,
    preflight_boolean_exact_request_from_graph, refinement_report_from_graph,
    validate_boolean_result_against_sources_with_artifacts,
    volumetric_boundary_closure_report_from_graph, winding_readiness_report_for_request_from_graph,
};
use super::cell_complex::{
    ExactRegionOwnershipReport, ExactRegionOwnershipStatus, ExactSelectedCellComplex,
    ExactSelectedCellComplexFreshness, select_arrangement_for_replay,
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
    exact_report_freshness,
};
use super::simplify::{ExactSimplifiedCellComplex, ExactSimplifiedCellComplexFreshness};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceError, CoplanarVolumetricCellEvidenceFreshness,
    CoplanarVolumetricCellEvidenceReport,
};

type MaterializedResultWithEvidence =
    Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>;
type OptionalMaterializedResult = Option<ExactBooleanResult>;

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
    closed_boundary_touching_regularized_materializations:
        Vec<(ExactBooleanRequest, MaterializedResultWithEvidence)>,
    closed_no_volume_overlap_regularized_materializations:
        Vec<(ExactBooleanRequest, MaterializedResultWithEvidence)>,
    open_surface_disjoint_materializations: Vec<(ExactBooleanRequest, OptionalMaterializedResult)>,
    boundary_touching_policy_materializations:
        Vec<(ExactBooleanRequest, OptionalMaterializedResult)>,
    closed_winding_containment_materializations:
        Vec<(ExactBooleanRequest, OptionalMaterializedResult)>,
    closed_winding_separated_materializations:
        Vec<(ExactBooleanRequest, OptionalMaterializedResult)>,
    adjacent_union_completion_materializations: Vec<(
        ExactBooleanRequest,
        Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>,
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
    certifications: Vec<(ExactBooleanRequest, ExactBooleanCertificationSet)>,
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
            open_surface_disjoint_materializations: Vec::new(),
            boundary_touching_policy_materializations: Vec::new(),
            closed_winding_containment_materializations: Vec::new(),
            closed_winding_separated_materializations: Vec::new(),
            adjacent_union_completion_materializations: Vec::new(),
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
            certifications: Vec::new(),
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

    fn validated_graph(&mut self) -> Result<&ExactIntersectionGraph, MeshError> {
        self.graph()?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        graph
            .validate_against_meshes(self.left, self.right)
            .map_err(workspace_graph_validation_error)?;
        Ok(graph)
    }

    fn validated_graph_with_sources(
        &mut self,
    ) -> Result<(&ExactIntersectionGraph, &'a ExactMesh, &'a ExactMesh), MeshError> {
        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        Ok((graph, left, right))
    }

    fn regularized_solid_arrangement(&self) -> Option<&ExactArrangement> {
        cached_by_policy_index(
            &self.arrangements,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .map(|index| &self.arrangements[index].1)
    }

    /// Returns retained coplanar volumetric-cell evidence, deriving it from
    /// the workspace's cached exact intersection graph.
    pub fn coplanar_volumetric_cell_evidence(
        &mut self,
    ) -> Result<&CoplanarVolumetricCellEvidenceReport, MeshError> {
        if self.coplanar_volumetric_cell_evidence.is_none() {
            let left = self.left;
            let right = self.right;
            let graph = self.validated_graph()?;
            let report = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
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

    /// Classify coplanar volumetric-cell evidence freshness in this retained
    /// source session.
    pub fn coplanar_volumetric_cell_evidence_freshness(
        &mut self,
        report: &CoplanarVolumetricCellEvidenceReport,
    ) -> CoplanarVolumetricCellEvidenceFreshness {
        report.freshness_against_sources(self.left, self.right)
    }

    /// Materializes zero-area closed boundary contact from the retained exact
    /// graph, caching both certified output and consumed coplanar evidence.
    pub fn materialize_closed_boundary_touching_regularized_with_evidence(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
        if let Some(cached) = cached_retained_result_with_evidence(
            &self.closed_boundary_touching_regularized_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized =
            materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                graph,
                left,
                right,
                request.operation,
                request.validation,
            )?;
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
        if let Some(cached) = cached_retained_result_with_evidence(
            &self.closed_no_volume_overlap_regularized_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized =
            materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                graph,
                left,
                right,
                request.operation,
                request.validation,
            )?;
        self.closed_no_volume_overlap_regularized_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes graph-disjoint open surfaces from the retained exact
    /// graph, caching certified output and declined outcomes per request.
    pub fn materialize_open_surface_disjoint(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        if let Some(cached) = cached_retained_optional_result(
            &self.open_surface_disjoint_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized =
            materialize_open_surface_disjoint_from_graph_for_request(graph, left, right, request)?;
        self.open_surface_disjoint_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes explicit boundary-only projection, preserving the public
    /// closed-validation shortcut and reusing the retained graph for
    /// boundary-policy replay.
    pub fn materialize_boundary_touching_policy(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        if let Some(cached) = cached_retained_optional_result(
            &self.boundary_touching_policy_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        if let Some(result) = boolean_closed_validation_regularized_meshes(
            self.left,
            self.right,
            request.operation,
            request.validation,
        )? {
            let materialized = Some(result);
            validate_retained_optional_result(&materialized, self.left, self.right, request)?;
            self.boundary_touching_policy_materializations
                .push((request, materialized.clone()));
            return Ok(materialized);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized = materialize_boundary_touching_policy_from_graph_for_request(
            graph, left, right, request,
        )?;
        self.boundary_touching_policy_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes closed-solid containment certified by exact winding and an
    /// empty retained graph.
    pub fn materialize_closed_winding_containment(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        if let Some(cached) = cached_retained_optional_result(
            &self.closed_winding_containment_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized = materialize_closed_winding_containment_from_graph_for_request(
            graph, left, right, request,
        )?;
        self.closed_winding_containment_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes closed-solid separation certified by exact winding and an
    /// empty retained graph.
    pub fn materialize_closed_winding_separated(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        if let Some(cached) = cached_retained_optional_result(
            &self.closed_winding_separated_materializations,
            self.left,
            self.right,
            request,
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized = materialize_closed_winding_separated_from_graph_for_request(
            graph, left, right, request,
        )?;
        self.closed_winding_separated_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Materializes adjacent closed-solid union completion from the retained
    /// graph, returning the consumed completion report.
    pub fn materialize_adjacent_union_completion(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>, MeshError> {
        if let Some(cached) = cached_materialization(
            &self.adjacent_union_completion_materializations,
            request,
            |cached| {
                validate_retained_result_with_adjacent_report(
                    cached, self.left, self.right, request,
                )
            },
        )? {
            return Ok(cached);
        }

        let (graph, left, right) = self.validated_graph_with_sources()?;
        let materialized = materialize_adjacent_union_completion_from_graph_for_request(
            graph, left, right, request,
        )?;
        self.adjacent_union_completion_materializations
            .push((request, materialized.clone()));
        Ok(materialized)
    }

    /// Returns the exact arrangement for `policy`, building it once per policy.
    pub fn arrangement(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangement, MeshError> {
        if let Some(index) = cached_by_policy_index(&self.arrangements, policy) {
            return Ok(&self.arrangements[index].1);
        }

        let graph = self.validated_graph()?.clone();
        let arrangement = ExactArrangement::from_intersection_graph_with_policy(
            graph, self.left, self.right, policy,
        )?;
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
        if let Some(index) = cached_by_policy_index(&self.topology_assembly_reports, policy) {
            return Ok(&self.topology_assembly_reports[index].1);
        }

        self.arrangement(policy)?;
        let arrangement_index = cached_by_policy_index(&self.arrangements, policy)
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
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        report.validate_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Classify topology-assembly evidence against this workspace's retained
    /// source session.
    pub fn topology_assembly_report_status(
        &mut self,
        policy: ExactRegularizationPolicy,
        report: &ExactTopologyAssemblyReport,
    ) -> ExactTopologyAssemblyStatus {
        let arrangement = match self.arrangement(policy) {
            Ok(arrangement) => arrangement.clone(),
            Err(_) => return ExactTopologyAssemblyStatus::SourceReplayBlocked,
        };
        report.status_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Returns region-ownership evidence for `policy`, reusing the cached
    /// arrangement and report for that policy.
    pub fn region_ownership_report(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactRegionOwnershipReport, MeshError> {
        if let Some(index) = cached_by_policy_index(&self.region_ownership_reports, policy) {
            return Ok(&self.region_ownership_reports[index].1);
        }

        self.arrangement(policy)?;
        let arrangement_index = cached_by_policy_index(&self.arrangements, policy)
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
        let arrangement = self
            .arrangement(policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .clone();
        report.validate_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Classify region-ownership evidence against this workspace's retained
    /// source session.
    pub fn region_ownership_report_status(
        &mut self,
        policy: ExactRegularizationPolicy,
        report: &ExactRegionOwnershipReport,
    ) -> ExactRegionOwnershipStatus {
        let arrangement = match self.arrangement(policy) {
            Ok(arrangement) => arrangement.clone(),
            Err(_) => return ExactRegionOwnershipStatus::SourceReplayBlocked,
        };
        report.status_against_arrangement(&arrangement, self.left, self.right, policy)
    }

    /// Returns the arrangement/cell-complex attempt report for `request` and
    /// `policy`, reusing the cached arrangement for that policy.
    pub fn arrangement_attempt(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangementBooleanAttempt, MeshError> {
        if let Some(index) =
            cached_by_request_and_policy_index(&self.arrangement_attempts, request, policy)
        {
            return Ok(&self.arrangement_attempts[index].2);
        }

        if let Some(attempt) =
            direct_arrangement_cell_complex_attempt(self.left, self.right, request, policy)?
        {
            self.arrangement_attempts.push((request, policy, attempt));
            return Ok(&self
                .arrangement_attempts
                .last()
                .expect("arrangement attempt cache was just populated")
                .2);
        }

        self.arrangement(policy)?;
        let arrangement_index = cached_by_policy_index(&self.arrangements, policy)
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
        if let Some(replay) =
            direct_arrangement_cell_complex_attempt(self.left, self.right, request, policy)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
        {
            attempt.validate()?;
            replay.validate()?;
            return if attempt == &replay {
                Ok(())
            } else {
                Err(ExactReportValidationError::SourceReplayMismatch)
            };
        }

        self.arrangement(policy)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        let arrangement_index = cached_by_policy_index(&self.arrangements, policy)
            .expect("arrangement cache was just populated");
        attempt.validate_against_arrangement(
            self.left,
            self.right,
            request,
            policy,
            &self.arrangements[arrangement_index].1,
        )
    }

    /// Classify arrangement/cell-complex attempt freshness in this retained
    /// source session.
    pub fn arrangement_attempt_freshness(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        attempt: &ExactArrangementBooleanAttempt,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_arrangement_attempt(request, policy, attempt))
    }

    /// Returns selected exact cell-complex evidence for `request` and `policy`,
    /// retaining the topology and ownership reports consumed by selection.
    pub fn selected_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactSelectedCellComplex, MeshError> {
        if let Some(index) =
            cached_by_request_and_policy_index(&self.selected_cell_complexes, request, policy)
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
            cached_by_request_and_policy_index(&self.simplified_cell_complexes, request, policy)
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
        if selected.operation != request.operation {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
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
        if selected.operation != request.operation {
            return ExactSelectedCellComplexFreshness::StaleSelectedCells;
        }
        let arrangement = match self.arrangement(policy) {
            Ok(arrangement) => arrangement.clone(),
            Err(_) => return ExactSelectedCellComplexFreshness::SourceReplayBlocked,
        };
        selected.freshness_against_arrangement(arrangement, self.left, self.right, policy)
    }

    /// Validate simplified cell-complex evidence against this workspace's
    /// retained source session.
    pub fn validate_simplified_cell_complex(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
        simplified: &ExactSimplifiedCellComplex,
    ) -> Result<(), ExactArrangementBlocker> {
        if simplified.operation != request.operation {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
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
        if simplified.operation != request.operation {
            return ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells;
        }
        let arrangement = match self.arrangement(policy) {
            Ok(arrangement) => arrangement.clone(),
            Err(_) => return ExactSimplifiedCellComplexFreshness::SourceReplayBlocked,
        };
        simplified.freshness_against_arrangement(arrangement, self.left, self.right, policy)
    }

    /// Returns preflight for `request`, building it once per request.
    pub fn preflight(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanPreflight, MeshError> {
        if let Some(index) = cached_by_request_index(&self.preflights, request) {
            return Ok(&self.preflights[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let preflight = preflight_boolean_exact_request_from_graph(graph, left, right, request)?;
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
        exact_report_freshness(self.validate_preflight(request, preflight))
    }

    /// Returns refinement evidence for `request`, building it once per
    /// request.
    pub fn refinement_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactRefinementReport, MeshError> {
        if let Some(index) = cached_by_request_index(&self.refinement_reports, request) {
            return Ok(&self.refinement_reports[index].1);
        }

        let graph = self.validated_graph()?;
        let report = refinement_report_from_graph(graph, request.operation);
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
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify refinement-report freshness in this retained source session.
    pub fn refinement_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactRefinementReport,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_refinement_report(request, report))
    }

    /// Returns adjacent-union completion evidence for `request`, building it
    /// once per request.
    pub fn adjacent_union_completion_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactAdjacentUnionCompletionReport, MeshError> {
        if let Some(index) =
            cached_by_request_index(&self.adjacent_union_completion_reports, request)
        {
            return Ok(&self.adjacent_union_completion_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let (report, _) = adjacent_union_completion_certification_from_graph(
            graph,
            left,
            right,
            request.operation,
            None,
        )?;
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
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify adjacent-union completion freshness in this retained source
    /// session.
    pub fn adjacent_union_completion_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactAdjacentUnionCompletionReport,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_adjacent_union_completion_report(request, report))
    }

    /// Returns identical-mesh evidence for `request`, building it once per
    /// request.
    pub fn identical_mesh_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> &ExactIdenticalMeshReport {
        if let Some(index) = cached_by_request_index(&self.identical_mesh_reports, request) {
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

    /// Classify identical-mesh freshness in this retained source session.
    pub fn identical_mesh_report_freshness(
        &mut self,
        _request: ExactBooleanRequest,
        report: &ExactIdenticalMeshReport,
    ) -> ExactReportFreshness {
        report.freshness_against_sources(self.left, self.right)
    }

    /// Returns same-surface evidence for `request`, building it once per
    /// request.
    pub fn same_surface_report(&mut self, request: ExactBooleanRequest) -> &ExactSameSurfaceReport {
        if let Some(index) = cached_by_request_index(&self.same_surface_reports, request) {
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

    /// Classify same-surface freshness in this retained source session.
    pub fn same_surface_report_freshness(
        &mut self,
        _request: ExactBooleanRequest,
        report: &ExactSameSurfaceReport,
    ) -> ExactReportFreshness {
        report.freshness_against_sources(self.left, self.right)
    }

    /// Returns boundary-touching evidence for `request`, building it once per
    /// request.
    pub fn boundary_touching_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBoundaryTouchingReport, MeshError> {
        if let Some(index) = cached_by_request_index(&self.boundary_touching_reports, request) {
            return Ok(&self.boundary_touching_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let report = boundary_touching_report_from_graph(graph, left, right)?;
        self.boundary_touching_reports.push((request, report));
        Ok(&self
            .boundary_touching_reports
            .last()
            .expect("boundary-touching report cache was just populated")
            .1)
    }

    /// Classify boundary-touching freshness in this retained source session.
    pub fn boundary_touching_report_freshness(
        &mut self,
        _request: ExactBooleanRequest,
        report: &ExactBoundaryTouchingReport,
    ) -> ExactReportFreshness {
        report.freshness_against_sources(self.left, self.right)
    }

    /// Returns open-surface disjointness evidence for `request`, building it
    /// once per request.
    pub fn open_surface_disjoint_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactOpenSurfaceDisjointReport, MeshError> {
        if let Some(index) = cached_by_request_index(&self.open_surface_disjoint_reports, request) {
            return Ok(&self.open_surface_disjoint_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let report = open_surface_disjoint_report_from_graph(graph, left, right);
        self.open_surface_disjoint_reports.push((request, report));
        Ok(&self
            .open_surface_disjoint_reports
            .last()
            .expect("open-surface disjoint report cache was just populated")
            .1)
    }

    /// Classify open-surface disjointness freshness in this retained source
    /// session.
    pub fn open_surface_disjoint_report_freshness(
        &mut self,
        _request: ExactBooleanRequest,
        report: &ExactOpenSurfaceDisjointReport,
    ) -> ExactReportFreshness {
        report.freshness_against_sources(self.left, self.right)
    }

    /// Returns volumetric boundary-closure evidence for `request`, building it
    /// once per request.
    pub fn volumetric_boundary_closure(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactVolumetricBoundaryClosureReport, MeshError> {
        if let Some(index) =
            cached_by_request_index(&self.volumetric_boundary_closure_reports, request)
        {
            return Ok(&self.volumetric_boundary_closure_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let report =
            volumetric_boundary_closure_report_from_graph(graph, left, right, request.operation)?;
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
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify volumetric boundary-closure freshness in this retained source
    /// session.
    pub fn volumetric_boundary_closure_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactVolumetricBoundaryClosureReport,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_volumetric_boundary_closure(request, report))
    }

    /// Returns winding-readiness evidence for `request`, building it once per
    /// request.
    pub fn winding_readiness(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactWindingReadinessReport, MeshError> {
        if let Some(index) = cached_by_request_index(&self.winding_readiness_reports, request) {
            return Ok(&self.winding_readiness_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let readiness =
            winding_readiness_report_for_request_from_graph(graph, left, right, request)?;
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
        exact_report_freshness(self.validate_winding_readiness(request, readiness))
    }

    /// Returns planar-arrangement readiness evidence for `request`, building
    /// it once per request.
    pub fn planar_arrangement_report(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactPlanarArrangementReport, MeshError> {
        if let Some(index) = cached_by_request_index(&self.planar_arrangement_reports, request) {
            return Ok(&self.planar_arrangement_reports[index].1);
        }

        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let report = planar_arrangement_report_from_graph(graph, left, right, request.operation)?;
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
        report.validate_against_sources(self.left, self.right)
    }

    /// Classify planar-arrangement readiness freshness in this retained source
    /// session.
    pub fn planar_arrangement_report_freshness(
        &mut self,
        request: ExactBooleanRequest,
        report: &ExactPlanarArrangementReport,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_planar_arrangement_report(request, report))
    }

    /// Returns the full exact boolean certification bundle for `request`,
    /// reusing retained graph and regularized arrangement artifacts.
    pub fn certification_set(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanCertificationSet, MeshError> {
        if let Some(index) = cached_by_request_index(&self.certifications, request) {
            return Ok(&self.certifications[index].1);
        }

        self.graph()?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
        let certifications = ExactBooleanCertificationSet::from_graph_and_regularized_arrangement(
            graph,
            self.left,
            self.right,
            request,
            regularized_arrangement,
        )?;
        certifications
            .validate_for_request(request)
            .map_err(workspace_report_validation_error)?;
        self.certifications.push((request, certifications));
        Ok(&self
            .certifications
            .last()
            .expect("certification cache was just populated")
            .1)
    }

    /// Validate a certification bundle against this workspace's retained graph
    /// and arrangement session.
    pub fn validate_certification_set(
        &mut self,
        request: ExactBooleanRequest,
        certifications: &ExactBooleanCertificationSet,
    ) -> Result<(), ExactReportValidationError> {
        self.graph()
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
        certifications.validate_against_sources_with_graph_and_regularized_arrangement(
            graph,
            self.left,
            self.right,
            request,
            regularized_arrangement,
        )
    }

    /// Classify certification-bundle freshness in this retained source
    /// session.
    pub fn certification_set_freshness(
        &mut self,
        request: ExactBooleanRequest,
        certifications: &ExactBooleanCertificationSet,
    ) -> ExactReportFreshness {
        exact_report_freshness(self.validate_certification_set(request, certifications))
    }

    /// Returns an exact boolean evaluation for `request`, building it once per
    /// request.
    pub fn evaluate(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanEvaluation, MeshError> {
        if let Some(index) = cached_by_request_index(&self.evaluations, request) {
            return Ok(&self.evaluations[index].1);
        }

        let preflight = self.preflight(request)?.clone();
        let certifications = self.certification_set(request)?.clone();
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
        let result = if preflight.is_certified() {
            if let Some(result) =
                cached_retained_result(&self.materializations, self.left, self.right, request)?
            {
                Some(result)
            } else {
                Some(materialize_certified_boolean_support_with_artifacts(
                    self.left,
                    self.right,
                    request,
                    preflight.support,
                    Some(graph),
                    regularized_arrangement,
                )?)
            }
        } else {
            None
        };
        let evaluation = ExactBooleanEvaluation {
            request,
            preflight,
            certifications,
            result,
        };
        evaluation
            .validate()
            .map_err(workspace_report_validation_error)?;
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
        if let Some(result) =
            cached_retained_result(&self.materializations, self.left, self.right, request)?
        {
            return Ok(result);
        }
        if let Some(evaluation) = self
            .evaluations
            .iter()
            .find(|(stored_request, _)| *stored_request == request)
            .map(|(_, evaluation)| evaluation)
            && let Some(result) = evaluation.result.as_ref()
        {
            evaluation
                .validate()
                .map_err(workspace_report_validation_error)?;
            if validate_retained_result_for_request(self.left, self.right, request, result).is_ok()
            {
                return Ok(result.clone());
            }
        }
        self.preflight(request)?;
        self.graph()?;
        let preflight_index = self
            .preflights
            .iter()
            .position(|(stored_request, _)| *stored_request == request)
            .expect("preflight cache was just populated");
        let preflight = &self.preflights[preflight_index].1;
        if preflight.is_certified() {
            let regularized_arrangement = self.regularized_solid_arrangement();
            let graph = self
                .graph
                .as_ref()
                .expect("intersection graph cache was just populated");
            let result = materialize_certified_boolean_support_with_artifacts(
                self.left,
                self.right,
                request,
                preflight.support,
                Some(graph),
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
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
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
        exact_report_freshness(self.validate_evaluation(evaluation))
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
            validate_retained_result_for_request(self.left, self.right, request, result)?;
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
        let regularized_arrangement = self.regularized_solid_arrangement();
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
        exact_report_freshness(self.validate_result(request, result))
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

fn validate_retained_result_with_evidence(
    materialized: &MaterializedResultWithEvidence,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<(), MeshError> {
    validate_retained_result_pair(materialized, left, right, request, |evidence| {
        evidence
            .validate_against_sources(left, right)
            .map_err(workspace_coplanar_volumetric_cell_error)
    })
}

fn validate_retained_result_with_adjacent_report(
    materialized: &Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<(), MeshError> {
    validate_retained_result_pair(materialized, left, right, request, |report| {
        report
            .validate_against_sources(left, right)
            .map_err(workspace_report_validation_error)
    })
}

fn validate_retained_result_pair<T>(
    materialized: &Option<(ExactBooleanResult, T)>,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    validate_artifact: impl FnOnce(&T) -> Result<(), MeshError>,
) -> Result<(), MeshError> {
    if let Some((result, artifact)) = materialized {
        validate_retained_result_for_request(left, right, request, result)
            .map_err(workspace_report_validation_error)?;
        validate_artifact(artifact)?;
    }
    Ok(())
}

fn validate_retained_optional_result(
    materialized: &OptionalMaterializedResult,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<(), MeshError> {
    if let Some(result) = materialized {
        validate_retained_result_for_request(left, right, request, result)
            .map_err(workspace_report_validation_error)?;
    }
    Ok(())
}

fn cached_retained_result(
    cache: &[(ExactBooleanRequest, ExactBooleanResult)],
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    cached_materialization(cache, request, |result| {
        validate_retained_result_for_request(left, right, request, result)
            .map_err(workspace_report_validation_error)
    })
}

fn cached_retained_optional_result(
    cache: &[(ExactBooleanRequest, OptionalMaterializedResult)],
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<OptionalMaterializedResult>, MeshError> {
    cached_materialization(cache, request, |result| {
        validate_retained_optional_result(result, left, right, request)
    })
}

fn cached_retained_result_with_evidence(
    cache: &[(ExactBooleanRequest, MaterializedResultWithEvidence)],
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<MaterializedResultWithEvidence>, MeshError> {
    cached_materialization(cache, request, |result| {
        validate_retained_result_with_evidence(result, left, right, request)
    })
}

fn cached_materialization<T: Clone>(
    cache: &[(ExactBooleanRequest, T)],
    request: ExactBooleanRequest,
    validate: impl FnOnce(&T) -> Result<(), MeshError>,
) -> Result<Option<T>, MeshError> {
    if let Some((_, cached)) = cache
        .iter()
        .find(|(stored_request, _)| *stored_request == request)
    {
        validate(cached)?;
        return Ok(Some(cached.clone()));
    }
    Ok(None)
}

fn cached_by_policy_index<T>(
    cache: &[(ExactRegularizationPolicy, T)],
    policy: ExactRegularizationPolicy,
) -> Option<usize> {
    cache
        .iter()
        .position(|(stored_policy, _)| *stored_policy == policy)
}

fn cached_by_request_index<T>(
    cache: &[(ExactBooleanRequest, T)],
    request: ExactBooleanRequest,
) -> Option<usize> {
    cache
        .iter()
        .position(|(stored_request, _)| *stored_request == request)
}

fn cached_by_request_and_policy_index<T>(
    cache: &[(ExactBooleanRequest, ExactRegularizationPolicy, T)],
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> Option<usize> {
    cache.iter().position(|(stored_request, stored_policy, _)| {
        *stored_request == request && *stored_policy == policy
    })
}

fn validate_retained_result_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    result: &ExactBooleanResult,
) -> Result<(), ExactReportValidationError> {
    if result.mesh.validation_policy() != request.validation || !result.matches_request(request) {
        return Err(ExactReportValidationError::StatusEvidenceMismatch);
    }
    result.validate_operation_against_sources(
        left,
        right,
        request.operation,
        request.validation,
        request.boundary_policy,
    )
}

fn workspace_report_validation_error(error: ExactReportValidationError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact boolean workspace retained report failed replay validation: {error:?}"),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::ExactBooleanOperation;
    use crate::validation::ValidationPolicy;
    use crate::{
        CoplanarVolumetricCellEvidenceError, CoplanarVolumetricCellEvidenceFreshness,
        ExactArrangementBooleanStage, ExactBooleanResultKind, ExactBooleanShortcutKind,
        ExactBoundaryBooleanPolicy, ExactReportValidationError, Triangle,
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
        coplanar_volumetric_evidence
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            workspace.coplanar_volumetric_cell_evidence_freshness(&coplanar_volumetric_evidence),
            CoplanarVolumetricCellEvidenceFreshness::Current
        );
        let mut stale_coplanar_volumetric_evidence = coplanar_volumetric_evidence.clone();
        stale_coplanar_volumetric_evidence.retained_face_pair_count += 1;
        assert_eq!(
            stale_coplanar_volumetric_evidence.validate_against_sources(&left, &right),
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
        assert_eq!(
            workspace.topology_assembly_report_status(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &topology_report,
            ),
            ExactTopologyAssemblyStatus::Complete
        );
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
        assert_eq!(
            workspace.topology_assembly_report_status(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_topology_report,
            ),
            ExactTopologyAssemblyStatus::StaleArrangement
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
        assert_eq!(
            workspace.region_ownership_report_status(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &ownership_report,
            ),
            ExactRegionOwnershipStatus::VolumeResolved
        );
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
        assert_eq!(
            workspace.region_ownership_report_status(
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_ownership_report,
            ),
            ExactRegionOwnershipStatus::StaleOwnership
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
        assert_eq!(
            workspace.arrangement_attempt_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_attempt,
            ),
            ExactReportFreshness::SourceReplayMismatch
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
        let mismatched_request =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED);
        assert_eq!(
            workspace.validate_selected_cell_complex(
                mismatched_request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &selected,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
        assert_eq!(
            workspace.selected_cell_complex_freshness(
                mismatched_request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &selected,
            ),
            ExactSelectedCellComplexFreshness::StaleSelectedCells
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
        assert_eq!(
            workspace.selected_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_selected,
            ),
            ExactSelectedCellComplexFreshness::StaleSelectedCells
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
        assert_eq!(
            workspace.validate_simplified_cell_complex(
                mismatched_request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &simplified,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
        assert_eq!(
            workspace.simplified_cell_complex_freshness(
                mismatched_request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &simplified,
            ),
            ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells
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
        assert_eq!(
            workspace.simplified_cell_complex_freshness(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_simplified,
            ),
            ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells
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
        identical_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            workspace.identical_mesh_report_freshness(request, &identical_report),
            ExactReportFreshness::Current
        );
        let mut stale_identical_report = identical_report.clone();
        stale_identical_report.left_triangles += 1;
        assert_eq!(
            stale_identical_report.validate_against_sources(&left, &right),
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
        same_surface_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            workspace.same_surface_report_freshness(request, &same_surface_report),
            ExactReportFreshness::Current
        );
        let mut stale_same_surface_report = same_surface_report.clone();
        stale_same_surface_report.predicates.clear();
        assert_eq!(
            stale_same_surface_report.validate_against_sources(&left, &right),
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
        boundary_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            workspace.boundary_touching_report_freshness(request, &boundary_report),
            ExactReportFreshness::Current
        );
        let mut stale_boundary_report = boundary_report.clone();
        stale_boundary_report.retained_events += 1;
        assert_eq!(
            stale_boundary_report.validate_against_sources(&left, &right),
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
        open_surface_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            workspace.open_surface_disjoint_report_freshness(request, &open_surface_report),
            ExactReportFreshness::Current
        );
        let mut stale_open_surface_report = open_surface_report.clone();
        stale_open_surface_report.left_open_surface = !stale_open_surface_report.left_open_surface;
        assert_eq!(
            stale_open_surface_report.validate_against_sources(&left, &right),
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
        materialized.validate().unwrap();
        materialized
            .validate_against_sources(&left, &right)
            .unwrap();
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
        workspace
            .materialize(request)
            .unwrap()
            .validate_against_sources(&left, &right)
            .unwrap();
    }

    #[test]
    fn exact_boolean_workspace_reuses_certification_set() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let first =
            workspace.certification_set(request).unwrap() as *const ExactBooleanCertificationSet;
        let second =
            workspace.certification_set(request).unwrap() as *const ExactBooleanCertificationSet;
        assert_eq!(first, second);

        let certifications = workspace.certification_set(request).unwrap().clone();
        certifications.validate_for_request(request).unwrap();
        certifications
            .validate_against_sources(&left, &right, request)
            .unwrap();
        workspace
            .validate_certification_set(request, &certifications)
            .unwrap();
        assert_eq!(
            workspace.certification_set_freshness(request, &certifications),
            ExactReportFreshness::Current
        );

        let mut stale = certifications;
        stale.refinement.retained_events += 1;
        assert_eq!(
            stale.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_eq!(
            workspace.validate_certification_set(request, &stale),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            workspace.certification_set_freshness(request, &stale),
            ExactReportFreshness::Current
        );
    }

    #[test]
    fn exact_boolean_workspace_arrangement_attempt_uses_orthogonal_shortcut_without_arrangement() {
        let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        assert_eq!(workspace.arrangements.len(), 0);
        assert_eq!(workspace.arrangement_attempts.len(), 1);
        assert_eq!(attempt.stage, ExactArrangementBooleanStage::Materialized);
        assert_eq!(
            attempt.materialized_shortcut,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex)
        );
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
        assert_eq!(workspace.arrangements.len(), 0);

        let evaluation = workspace.evaluate(request).unwrap().clone();
        evaluation.validate().unwrap();
        assert!(evaluation.preflight.is_certified());
        assert!(evaluation.result.is_some());
        assert!(evaluation.certifications.topology_assembly.is_none());
        assert!(evaluation.certifications.region_ownership.is_none());
        assert!(evaluation.certifications.arrangement_attempt.is_some());
        assert_eq!(workspace.arrangements.len(), 0);

        let result = workspace.materialize(request).unwrap();
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            }
        );
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(workspace.arrangements.len(), 0);

        let mut stale_attempt = attempt.clone();
        stale_attempt.output_triangles += 1;
        stale_attempt.validate().unwrap();
        assert_eq!(
            workspace.validate_arrangement_attempt(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_attempt,
            ),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_eq!(workspace.arrangements.len(), 0);
    }

    #[test]
    fn exact_boolean_workspace_validates_cached_evaluation_locally() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let retained = workspace.evaluate(request).unwrap().clone();
        workspace.validate_evaluation(&retained).unwrap();
        assert_eq!(
            workspace.evaluation_freshness(&retained),
            ExactReportFreshness::Current
        );

        let mut stale = retained.clone();
        stale.preflight.retained_events += 1;
        assert_eq!(
            workspace.validate_evaluation(&stale),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            workspace.evaluation_freshness(&stale),
            ExactReportFreshness::Current
        );

        let mut corrupt_proof_workspace = ExactBooleanWorkspace::new(&left, &right);
        corrupt_proof_workspace.evaluate(request).unwrap();
        corrupt_proof_workspace.evaluations[0]
            .1
            .preflight
            .retained_events += 1;
        assert!(
            corrupt_proof_workspace.materialize(request).is_err(),
            "cached evaluation proof must validate before materialization reuse"
        );

        let cached_result = workspace.evaluations[0]
            .1
            .result
            .as_mut()
            .expect("certified test request should retain a result");
        cached_result.graph_had_unknowns = !cached_result.graph_had_unknowns;
        let corrupted = workspace.evaluations[0].1.clone();
        assert!(
            workspace.validate_evaluation(&corrupted).is_err(),
            "cached evaluation validation must still enforce local invariants"
        );
        assert!(
            workspace.materialize(request).is_err(),
            "cached evaluation results must validate before materialization reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_evaluation_reuses_materialization_cache() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let materialized = workspace.materialize(request).unwrap();
        assert_eq!(workspace.materializations.len(), 1);
        let evaluation = workspace.evaluate(request).unwrap().clone();
        assert_eq!(evaluation.result.as_ref(), Some(&materialized));
        assert_eq!(workspace.materializations.len(), 1);
        evaluation.validate().unwrap();

        let mut corrupt_workspace = ExactBooleanWorkspace::new(&left, &right);
        corrupt_workspace.materialize(request).unwrap();
        corrupt_workspace.materializations[0].1.graph_had_unknowns =
            !corrupt_workspace.materializations[0].1.graph_had_unknowns;
        assert!(
            corrupt_workspace.evaluate(request).is_err(),
            "cached materialization must validate before evaluation reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_rejects_relabelled_cached_materialization() {
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
        let request = ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        );
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace.materialize(request).unwrap();
        assert_eq!(
            materialized.kind,
            ExactBooleanResultKind::BoundaryPolicyShortcut {
                operation: ExactBooleanOperation::Union
            }
        );
        workspace.materializations[0].1.kind = ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::Difference,
        };
        let relabelled = workspace.materializations[0].1.clone();
        assert!(
            workspace.validate_result(request, &relabelled).is_err(),
            "cached result validation must reject relabelled operations"
        );
        assert!(
            workspace.materialize(request).is_err(),
            "cached materialization must match the request operation before reuse"
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

        let mut relabelled = materialized.clone();
        relabelled.0.kind = ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Difference,
            shortcut: ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        };
        workspace.closed_boundary_touching_regularized_materializations[0].1 = Some(relabelled);
        assert!(
            workspace
                .materialize_closed_boundary_touching_regularized_with_evidence(request)
                .is_err(),
            "cached boundary-touching materialization must match the request operation"
        );
        workspace.closed_boundary_touching_regularized_materializations[0].1 =
            Some(materialized.clone());

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

    #[test]
    fn exact_boolean_workspace_reuses_open_surface_disjoint_materialization() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 4, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 1, 4, 0, 5, 0, 4, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let request = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        );
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_open_surface_disjoint(request)
            .unwrap()
            .expect("graph-disjoint open surfaces should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_open_surface_disjoint(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(workspace.open_surface_disjoint_materializations.len(), 1);
        assert_eq!(
            workspace
                .materialize_open_surface_disjoint(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(workspace.open_surface_disjoint_materializations.len(), 1);

        let cached = workspace.open_surface_disjoint_materializations[0]
            .1
            .as_mut()
            .unwrap();
        cached.graph_had_unknowns = !cached.graph_had_unknowns;
        assert!(
            workspace
                .materialize_open_surface_disjoint(request)
                .is_err(),
            "cached open-surface disjoint materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_boundary_touching_policy_materialization() {
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
        let request = ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        );
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_boundary_touching_policy(request)
            .unwrap()
            .expect("boundary policy should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_boundary_touching_policy(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(workspace.boundary_touching_policy_materializations.len(), 1);
        assert_eq!(
            workspace
                .materialize_boundary_touching_policy(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(workspace.boundary_touching_policy_materializations.len(), 1);

        let mut relabelled = materialized.clone();
        relabelled.kind = ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::Difference,
        };
        workspace.boundary_touching_policy_materializations[0].1 = Some(relabelled);
        assert!(
            workspace
                .materialize_boundary_touching_policy(request)
                .is_err(),
            "cached boundary-policy materialization must match the request operation"
        );
        workspace.boundary_touching_policy_materializations[0].1 = Some(materialized.clone());

        let cached = workspace.boundary_touching_policy_materializations[0]
            .1
            .as_mut()
            .unwrap();
        cached.graph_had_unknowns = !cached.graph_had_unknowns;
        assert!(
            workspace
                .materialize_boundary_touching_policy(request)
                .is_err(),
            "cached boundary-policy materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_winding_containment_materialization() {
        let outer = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
        let disjoint_shell = tetra_from_corners([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]);
        let container = combine_exact_meshes(
            &outer,
            &disjoint_shell,
            "workspace disconnected winding container",
        );
        let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&container, &contained);

        let materialized = workspace
            .materialize_closed_winding_containment(request)
            .unwrap()
            .expect("empty graph containment should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_closed_winding_containment(&container, &contained)
                .unwrap()
                .unwrap()
        );
        assert_eq!(
            workspace.closed_winding_containment_materializations.len(),
            1
        );
        assert_eq!(
            workspace
                .materialize_closed_winding_containment(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(
            workspace.closed_winding_containment_materializations.len(),
            1
        );

        let cached = workspace.closed_winding_containment_materializations[0]
            .1
            .as_mut()
            .unwrap();
        cached.graph_had_unknowns = !cached.graph_had_unknowns;
        assert!(
            workspace
                .materialize_closed_winding_containment(request)
                .is_err(),
            "cached closed-winding containment materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_winding_separation_materialization() {
        let left_a = tetra_from_corners([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let left_b = tetra_from_corners([10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]);
        let left = combine_exact_meshes(
            &left_a,
            &left_b,
            "workspace disconnected winding separated left",
        );
        let right = tetra_from_corners([5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_closed_winding_separated(request)
            .unwrap()
            .expect("empty graph separation should materialize from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_closed_winding_separated(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(workspace.closed_winding_separated_materializations.len(), 1);
        assert_eq!(
            workspace
                .materialize_closed_winding_separated(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(workspace.closed_winding_separated_materializations.len(), 1);

        let cached = workspace.closed_winding_separated_materializations[0]
            .1
            .as_mut()
            .unwrap();
        cached.graph_had_unknowns = !cached.graph_had_unknowns;
        assert!(
            workspace
                .materialize_closed_winding_separated(request)
                .is_err(),
            "cached closed-winding separation materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_adjacent_union_completion_materialization() {
        let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
        let left = combine_exact_meshes(
            &left_a,
            &left_b,
            "workspace disconnected full-face adjacent fixture",
        );
        let right = tetra_from_corners([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let materialized = workspace
            .materialize_adjacent_union_completion(request)
            .unwrap()
            .expect("adjacent closed solids should complete from retained graph");
        assert_eq!(
            materialized,
            request
                .materialize_adjacent_union_completion(&left, &right)
                .unwrap()
                .unwrap()
        );
        assert_eq!(
            workspace.adjacent_union_completion_materializations.len(),
            1
        );
        assert_eq!(
            workspace
                .materialize_adjacent_union_completion(request)
                .unwrap()
                .unwrap(),
            materialized
        );
        assert_eq!(
            workspace.adjacent_union_completion_materializations.len(),
            1
        );

        let cached = &mut workspace.adjacent_union_completion_materializations[0]
            .1
            .as_mut()
            .unwrap()
            .0;
        cached.graph_had_unknowns = !cached.graph_had_unknowns;
        assert!(
            workspace
                .materialize_adjacent_union_completion(request)
                .is_err(),
            "cached adjacent-union materialization must validate before reuse"
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

    fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        let [x0, y0, z0] = min;
        let [x1, y1, z1] = max;
        ExactMesh::from_i64_triangles(
            &[
                x0, y0, z0, x1, y0, z0, x1, y1, z0, x0, y1, z0, x0, y0, z1, x1, y0, z1, x1, y1, z1,
                x0, y1, z1,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
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
