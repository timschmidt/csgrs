use super::arrangement3d::{ExactArrangement, ExactTopologyAssemblyReport};
use super::boolean::{
    arrangement_boolean_attempt_report_from_arrangement,
    evaluate_boolean_exact_request_with_artifacts_and_arrangement_replay,
    materialize_certified_boolean_support_with_arrangement,
    validate_boolean_result_against_sources_with_artifacts, ExactArrangementBooleanAttempt,
    ExactBooleanEvaluation, ExactBooleanRequest,
};
use super::cell_complex::{
    select_arrangement_for_replay, ExactRegionOwnershipReport, ExactSelectedCellComplex,
    ExactSelectedCellComplexFreshness,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{build_intersection_graph, ExactIntersectionGraph};
use super::mesh::ExactMesh;
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::reports::{
    ExactBooleanPreflight, ExactBooleanResult, ExactReportFreshness, ExactReportValidationError,
};
use super::simplify::{ExactSimplifiedCellComplex, ExactSimplifiedCellComplexFreshness};

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
    preflights: Vec<(ExactBooleanRequest, ExactBooleanPreflight)>,
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
            arrangements: Vec::new(),
            topology_assembly_reports: Vec::new(),
            region_ownership_reports: Vec::new(),
            arrangement_attempts: Vec::new(),
            selected_cell_complexes: Vec::new(),
            simplified_cell_complexes: Vec::new(),
            preflights: Vec::new(),
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
    use crate::{ExactBooleanResultKind, ExactReportValidationError};

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
        assert!(workspace
            .validate_arrangement_attempt(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_attempt,
            )
            .is_err());
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
        assert!(workspace
            .validate_selected_cell_complex(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_selected,
            )
            .is_err());
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
        assert!(workspace
            .validate_simplified_cell_complex(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                &stale_simplified,
            )
            .is_err());
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
        assert!(materialize_workspace
            .validate_result(request, &locally_invalid_cached_result)
            .is_err());
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
        assert!(materialize_workspace
            .validate_result(request, &stale_result)
            .is_err());
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

    fn tetra(offset: [i64; 3]) -> ExactMesh {
        let [ox, oy, oz] = offset;
        ExactMesh::from_i64_triangles(
            &[ox, oy, oz, ox + 1, oy, oz, ox, oy + 1, oz, ox, oy, oz + 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }
}
