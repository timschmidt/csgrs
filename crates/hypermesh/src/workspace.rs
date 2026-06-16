use super::arrangement3d::ExactArrangement;
use super::boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanCertificationSet, ExactBooleanEvaluation,
    ExactBooleanRequest, arrangement_boolean_attempt_report_from_arrangement,
    arrangement_cell_complex_shortcut_attempt,
    materialize_boolean_exact_request_from_retained_graph,
    materialize_certified_boolean_support_with_artifacts,
    preflight_boolean_exact_request_from_graph,
    try_materialize_certified_boolean_support_with_artifacts,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{ExactIntersectionGraph, build_intersection_graph};
use super::mesh::ExactMesh;
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::reports::{ExactBooleanPreflight, ExactBooleanResult, ExactReportValidationError};

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
    arrangement_attempts: Vec<(
        ExactBooleanRequest,
        ExactRegularizationPolicy,
        ExactArrangementBooleanAttempt,
    )>,
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
            arrangement_attempts: Vec::new(),
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
        let left = self.left;
        let right = self.right;
        let graph = self.graph()?;
        graph
            .validate_against_meshes(left, right)
            .map_err(|error| {
                MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    format!("exact boolean workspace graph failed validation: {error:?}"),
                ))
            })?;
        Ok(graph)
    }

    fn regularized_solid_arrangement(&self) -> Option<&ExactArrangement> {
        cached_by_policy_index(
            &self.arrangements,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .map(|index| &self.arrangements[index].1)
    }

    fn regularized_solid_arrangement_attempt(
        &self,
        request: ExactBooleanRequest,
    ) -> Option<&ExactArrangementBooleanAttempt> {
        cached_by_request_and_policy_index(
            &self.arrangement_attempts,
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .map(|index| &self.arrangement_attempts[index].2)
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
        store_retained_policy_artifact(&mut self.arrangements, policy, arrangement)
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
            arrangement_cell_complex_shortcut_attempt(self.left, self.right, request, policy)?
        {
            return store_retained_arrangement_attempt(
                &mut self.arrangement_attempts,
                request,
                policy,
                attempt,
            );
        }

        let left = self.left;
        let right = self.right;
        let arrangement = self.arrangement(policy)?;
        let attempt = arrangement_boolean_attempt_report_from_arrangement(
            left,
            right,
            request,
            policy,
            arrangement,
        )?;
        store_retained_arrangement_attempt(&mut self.arrangement_attempts, request, policy, attempt)
    }

    /// Derive preflight for `request` from the retained graph.
    fn preflight(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanPreflight, MeshError> {
        let left = self.left;
        let right = self.right;
        let graph = self.validated_graph()?;
        let preflight = preflight_boolean_exact_request_from_graph(graph, left, right, request)?;
        if preflight.operation != request.operation {
            return Err(workspace_report_validation_error(
                ExactReportValidationError::StatusEvidenceMismatch,
            ));
        }
        preflight
            .validate()
            .map_err(workspace_report_validation_error)?;
        Ok(preflight)
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

        let preflight = self.preflight(request)?;
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
        let regularized_attempt = self.regularized_solid_arrangement_attempt(request);
        let certifications = ExactBooleanCertificationSet::from_graph_and_regularized_arrangement(
            graph,
            self.left,
            self.right,
            request,
            regularized_arrangement,
            regularized_attempt,
        )?;
        let result = if preflight.is_certified() {
            if let Some(result) = cached_retained_materialization(
                &self.materializations,
                self.left,
                self.right,
                request,
            )? {
                Some(result)
            } else {
                try_materialize_certified_boolean_support_with_artifacts(
                    self.left,
                    self.right,
                    request,
                    preflight.support,
                    Some(graph),
                    regularized_arrangement,
                )?
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
        store_retained_request_artifact(&mut self.evaluations, request, evaluation)
    }

    /// Materializes `request`, reusing a cached certified evaluation when the
    /// workspace has one and otherwise reusing retained graph, preflight, and
    /// arrangement artifacts for certified supports.
    pub fn materialize(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanResult, MeshError> {
        if let Some(result) =
            cached_retained_materialization(&self.materializations, self.left, self.right, request)?
        {
            self.promote_evaluation_cache_from_materialization(request, &result)?;
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
                let result = store_replayable_result_or_return(
                    &mut self.materializations,
                    self.left,
                    self.right,
                    request,
                    result.clone(),
                )?;
                self.promote_evaluation_cache_from_materialization(request, &result)?;
                return Ok(result);
            }
        }
        let preflight = self.preflight(request)?;
        if preflight.is_certified() {
            let evaluation = self.evaluate(request)?.clone();
            evaluation
                .validate()
                .map_err(workspace_report_validation_error)?;
            if let Some(result) = evaluation.result {
                let result = store_replayable_result_or_return(
                    &mut self.materializations,
                    self.left,
                    self.right,
                    request,
                    result,
                )?;
                self.promote_evaluation_cache_from_materialization(request, &result)?;
                return Ok(result);
            }
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
            let result = store_replayable_result_or_return(
                &mut self.materializations,
                self.left,
                self.right,
                request,
                result,
            )?;
            self.promote_evaluation_cache_from_materialization(request, &result)?;
            return Ok(result);
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let result = materialize_boolean_exact_request_from_retained_graph(
            graph, self.left, self.right, request,
        )?;
        let result = store_replayable_result_or_return(
            &mut self.materializations,
            self.left,
            self.right,
            request,
            result,
        )?;
        self.promote_evaluation_cache_from_materialization(request, &result)?;
        Ok(result)
    }

    fn promote_evaluation_cache_from_materialization(
        &mut self,
        request: ExactBooleanRequest,
        result: &ExactBooleanResult,
    ) -> Result<(), MeshError> {
        if cached_by_request_index(&self.materializations, request).is_none() {
            return Ok(());
        }
        validate_retained_result_for_request(self.left, self.right, request, result)
            .map_err(workspace_report_validation_error)?;
        if let Some(index) = cached_by_request_index(&self.evaluations, request) {
            let evaluation = &mut self.evaluations[index].1;
            evaluation
                .validate()
                .map_err(workspace_report_validation_error)?;
            match evaluation.result.as_ref() {
                Some(existing) if existing == result => Ok(()),
                Some(_) => Err(workspace_report_validation_error(
                    ExactReportValidationError::StatusEvidenceMismatch,
                )),
                None => {
                    evaluation.result = Some(result.clone());
                    evaluation
                        .validate()
                        .map_err(workspace_report_validation_error)
                }
            }
        } else {
            let evaluation = self.evaluate(request)?;
            if evaluation.result.as_ref() == Some(result) {
                Ok(())
            } else {
                Err(workspace_report_validation_error(
                    ExactReportValidationError::StatusEvidenceMismatch,
                ))
            }
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

trait RetainedPolicyArtifact {
    fn validate_for_workspace_cache(&self) -> Result<(), ExactArrangementBlocker>;
}

trait RetainedRequestArtifact {
    fn validate_for_workspace_cache(
        &self,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError>;
}

impl RetainedPolicyArtifact for ExactArrangement {
    fn validate_for_workspace_cache(&self) -> Result<(), ExactArrangementBlocker> {
        self.validate()
    }
}

impl RetainedRequestArtifact for ExactBooleanEvaluation {
    fn validate_for_workspace_cache(
        &self,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError> {
        if self.request != request {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        self.validate()
    }
}

trait RetainedMaterializationCacheValue: Clone {
    fn validate_for_workspace_cache(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), MeshError>;
}

impl RetainedMaterializationCacheValue for ExactBooleanResult {
    fn validate_for_workspace_cache(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), MeshError> {
        validate_retained_result_for_request(left, right, request, self)
            .map_err(workspace_report_validation_error)?;
        Ok(())
    }
}

fn store_replayable_result_or_return(
    cache: &mut Vec<(ExactBooleanRequest, ExactBooleanResult)>,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    result: ExactBooleanResult,
) -> Result<ExactBooleanResult, MeshError> {
    if result
        .validate_for_workspace_cache(left, right, request)
        .is_ok()
    {
        cache.push((request, result.clone()));
    } else {
        result
            .validate()
            .map_err(workspace_report_validation_error)?;
        if !result
            .mesh
            .validation_policy()
            .satisfies(request.validation)
            || !result.matches_request(request)
        {
            return Err(workspace_report_validation_error(
                ExactReportValidationError::StatusEvidenceMismatch,
            ));
        }
    }
    Ok(result)
}

fn store_retained_policy_artifact<T: RetainedPolicyArtifact>(
    cache: &mut Vec<(ExactRegularizationPolicy, T)>,
    policy: ExactRegularizationPolicy,
    report: T,
) -> Result<&T, MeshError> {
    report
        .validate_for_workspace_cache()
        .map_err(workspace_arrangement_blocker_error)?;
    cache.push((policy, report));
    Ok(&cache
        .last()
        .expect("policy report cache was just populated")
        .1)
}

fn store_retained_arrangement_attempt(
    cache: &mut Vec<(
        ExactBooleanRequest,
        ExactRegularizationPolicy,
        ExactArrangementBooleanAttempt,
    )>,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    attempt: ExactArrangementBooleanAttempt,
) -> Result<&ExactArrangementBooleanAttempt, MeshError> {
    if attempt.operation != request.operation
        || attempt.output_validation != request.validation
        || attempt.policy != policy
    {
        return Err(workspace_report_validation_error(
            ExactReportValidationError::StatusEvidenceMismatch,
        ));
    }
    attempt
        .validate()
        .map_err(workspace_report_validation_error)?;
    cache.push((request, policy, attempt));
    Ok(&cache
        .last()
        .expect("arrangement attempt cache was just populated")
        .2)
}

fn store_retained_request_artifact<T: RetainedRequestArtifact>(
    cache: &mut Vec<(ExactBooleanRequest, T)>,
    request: ExactBooleanRequest,
    artifact: T,
) -> Result<&T, MeshError> {
    artifact
        .validate_for_workspace_cache(request)
        .map_err(workspace_report_validation_error)?;
    cache.push((request, artifact));
    Ok(&cache
        .last()
        .expect("request artifact cache was just populated")
        .1)
}

fn cached_retained_materialization<T: RetainedMaterializationCacheValue>(
    cache: &[(ExactBooleanRequest, T)],
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<T>, MeshError> {
    if let Some(index) = cached_by_request_index(cache, request) {
        cache[index]
            .1
            .validate_for_workspace_cache(left, right, request)?;
        return Ok(Some(cache[index].1.clone()));
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
    if !result
        .mesh
        .validation_policy()
        .satisfies(request.validation)
        || !result.matches_request(request)
    {
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
    use crate::boolean::{
        ExactBooleanOperation, identical_mesh_report_from_sources, same_surface_report_from_sources,
    };
    use crate::region::ExactRegionSelection;
    use crate::reports::exact_report_freshness;
    use crate::validation::ValidationPolicy;
    use crate::{
        ExactAdjacentUnionCompletionStatus, ExactArrangementBooleanStage, ExactBooleanResultKind,
        ExactBooleanShortcutKind, ExactBoundaryBooleanPolicy, ExactRegionOwnershipStatus,
        ExactReportFreshness, ExactReportValidationError, ExactSelectedCellComplexFreshness,
        ExactSimplifiedCellComplexFreshness, ExactTopologyAssemblyStatus, Triangle,
    };

    fn workspace_certifications(
        workspace: &mut ExactBooleanWorkspace<'_>,
        request: ExactBooleanRequest,
    ) -> ExactBooleanCertificationSet {
        workspace.graph().unwrap();
        let graph = workspace
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        ExactBooleanCertificationSet::from_graph_and_regularized_arrangement(
            graph,
            workspace.left,
            workspace.right,
            request,
            workspace.regularized_solid_arrangement(),
            workspace.regularized_solid_arrangement_attempt(request),
        )
        .unwrap()
    }

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

        let attempt_with_reports = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        let topology_report = attempt_with_reports
            .topology_assembly_report
            .clone()
            .expect("arrangement attempt should retain topology evidence");
        topology_report.validate().unwrap();
        topology_report
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        assert_eq!(
            topology_report.status_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
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
            stale_topology_report.status_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            ExactTopologyAssemblyStatus::StaleArrangement
        );

        let ownership_report = attempt_with_reports
            .region_ownership_report
            .clone()
            .expect("arrangement attempt should retain ownership evidence");
        ownership_report.validate().unwrap();
        ownership_report
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        assert_eq!(
            ownership_report.status_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
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
            stale_ownership_report.status_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
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
        let mut replay_workspace = ExactBooleanWorkspace::new(&left, &right);
        assert_eq!(
            workspace
                .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap(),
            replay_workspace
                .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap()
        );
        let attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
            .clone();
        let attempt_selected = attempt
            .selected_cell_complex
            .clone()
            .expect("generic arrangement attempt should retain selected cells");
        let attempt_simplified = attempt
            .simplified_cell_complex
            .clone()
            .expect("generic arrangement attempt should retain simplified cells");
        attempt.validate().unwrap();
        attempt.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            attempt.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_attempt = attempt.clone();
        stale_attempt
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        if let Some(selected) = stale_attempt.selected_cell_complex.as_mut() {
            selected
                .topology_assembly_report
                .as_mut()
                .unwrap()
                .graph_events += 1;
        }
        if let Some(simplified) = stale_attempt.simplified_cell_complex.as_mut() {
            simplified
                .topology_assembly_report
                .as_mut()
                .unwrap()
                .graph_events += 1;
        }
        stale_attempt.validate().unwrap();
        assert_eq!(
            stale_attempt.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch
        );

        let selected = attempt_selected;
        assert!(selected.topology_assembly_report.is_some());
        assert!(selected.region_ownership_report.is_some());
        selected.validate().unwrap();
        selected
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        assert_eq!(
            selected.freshness_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            ExactSelectedCellComplexFreshness::Current
        );
        let mismatched_request =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED);
        assert_ne!(
            selected.operation, mismatched_request.operation,
            "selected artifact should carry the request operation it proves"
        );
        let mut stale_selected = selected.clone();
        stale_selected
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        assert_eq!(
            stale_selected.freshness_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            ExactSelectedCellComplexFreshness::StaleSelectedCells
        );

        let simplified = attempt_simplified;
        assert!(simplified.topology_assembly_report.is_some());
        assert!(simplified.region_ownership_report.is_some());
        simplified.validate().unwrap();
        simplified
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        assert_eq!(
            simplified.freshness_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            ExactSimplifiedCellComplexFreshness::Current
        );
        assert_ne!(
            simplified.operation, mismatched_request.operation,
            "simplified artifact should carry the request operation it proves"
        );
        let mut stale_simplified = simplified.clone();
        stale_simplified
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;
        assert_eq!(
            stale_simplified.freshness_against_sources(
                &left,
                &right,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells
        );

        let first_preflight = workspace.preflight(request).unwrap();
        let second_preflight = workspace.preflight(request).unwrap();
        assert_eq!(first_preflight, second_preflight);
        assert_eq!(first_preflight, request.preflight(&left, &right).unwrap());
        let preflight = first_preflight;
        preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy,
            )
            .unwrap();
        assert_eq!(
            preflight.freshness_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy
            ),
            ExactReportFreshness::Current
        );
        let mut stale_preflight = preflight.clone();
        stale_preflight.retained_events += 1;
        assert_eq!(
            stale_preflight.validate_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy,
            ),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            stale_preflight.freshness_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy
            ),
            ExactReportFreshness::Current
        );
        let mut relabeled_preflight = preflight.clone();
        relabeled_preflight.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_preflight.validate_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy,
            ),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let certifications = workspace_certifications(&mut workspace, request);
        certifications
            .validate_against_sources(&left, &right, request)
            .unwrap();
        assert_eq!(certifications.arrangement_attempt.as_ref(), Some(&attempt));

        let refinement_report = certifications.refinement.clone();
        assert_eq!(
            refinement_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_refinement_bundle = certifications.clone();
        stale_refinement_bundle.refinement.retained_events += 1;
        assert_eq!(
            stale_refinement_bundle.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut relabeled_refinement_bundle = certifications.clone();
        relabeled_refinement_bundle.refinement.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_refinement_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let adjacent_report = certifications.adjacent_union_completion.clone();
        assert_eq!(
            adjacent_report,
            crate::boolean::adjacent_union_completion_certification(
                &left,
                &right,
                request.operation,
                None,
            )
            .unwrap()
            .0
        );
        assert_eq!(
            adjacent_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_adjacent_bundle = certifications.clone();
        stale_adjacent_bundle
            .adjacent_union_completion
            .stronger_kernel_available = !stale_adjacent_bundle
            .adjacent_union_completion
            .stronger_kernel_available;
        assert_eq!(
            stale_adjacent_bundle.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut relabeled_adjacent_bundle = certifications.clone();
        relabeled_adjacent_bundle
            .adjacent_union_completion
            .operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_adjacent_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let non_union_request = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        );
        let non_union_adjacent_report =
            workspace_certifications(&mut workspace, non_union_request).adjacent_union_completion;
        assert_eq!(
            non_union_adjacent_report,
            crate::boolean::adjacent_union_completion_certification(
                &left,
                &right,
                non_union_request.operation,
                None
            )
            .unwrap()
            .0
        );
        workspace_certifications(&mut workspace, non_union_request)
            .validate_for_request(non_union_request)
            .unwrap();
        assert_eq!(non_union_adjacent_report.retained_face_pairs, 0);
        assert_eq!(non_union_adjacent_report.retained_events, 0);
        let open_sheet = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, //
                1, 0, 0, //
                0, 1, 0,
            ],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mut open_workspace = ExactBooleanWorkspace::new(&open_sheet, &right);
        let not_closed_request = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        );
        let not_closed_adjacent_report =
            workspace_certifications(&mut open_workspace, not_closed_request)
                .adjacent_union_completion;
        assert_eq!(
            not_closed_adjacent_report,
            crate::boolean::adjacent_union_completion_certification(
                &open_sheet,
                &right,
                not_closed_request.operation,
                None
            )
            .unwrap()
            .0
        );
        workspace_certifications(&mut open_workspace, not_closed_request)
            .validate_for_request(not_closed_request)
            .unwrap();
        assert_eq!(not_closed_adjacent_report.retained_face_pairs, 0);
        assert_eq!(not_closed_adjacent_report.retained_events, 0);
        let box_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let box_right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let mut box_workspace = ExactBooleanWorkspace::new(&box_left, &box_right);
        let axis_box_adjacent_report =
            workspace_certifications(&mut box_workspace, request).adjacent_union_completion;
        assert_eq!(
            axis_box_adjacent_report,
            crate::boolean::adjacent_union_completion_certification(
                &box_left,
                &box_right,
                request.operation,
                None,
            )
            .unwrap()
            .0
        );
        workspace_certifications(&mut box_workspace, request)
            .validate_for_request(request)
            .unwrap();
        assert_eq!(
            axis_box_adjacent_report.status,
            ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair
        );
        assert_eq!(axis_box_adjacent_report.retained_face_pairs, 0);
        assert_eq!(axis_box_adjacent_report.retained_events, 0);

        let identical_report = identical_mesh_report_from_sources(&left, &right);
        identical_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            identical_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_identical_report = identical_report.clone();
        stale_identical_report.left_triangles += 1;
        assert_eq!(
            stale_identical_report.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            stale_identical_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let same_surface_report = same_surface_report_from_sources(&left, &right);
        same_surface_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            same_surface_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_same_surface_report = same_surface_report.clone();
        stale_same_surface_report.predicates.clear();
        assert_eq!(
            stale_same_surface_report.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            stale_same_surface_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let boundary_report = certifications.boundary_touching.clone();
        boundary_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            boundary_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_boundary_report = boundary_report.clone();
        stale_boundary_report.retained_events += 1;
        assert_eq!(
            stale_boundary_report.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            stale_boundary_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let open_surface_report = certifications.open_surface_disjoint.clone();
        open_surface_report
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            open_surface_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_open_surface_report = open_surface_report.clone();
        stale_open_surface_report.left_open_surface = !stale_open_surface_report.left_open_surface;
        assert_eq!(
            stale_open_surface_report.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            stale_open_surface_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let closure_report = certifications.volumetric_boundary_closure.clone().unwrap();
        assert_eq!(
            closure_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_closure_bundle = certifications.clone();
        stale_closure_bundle
            .volumetric_boundary_closure
            .as_mut()
            .unwrap()
            .output_triangles += 1;
        assert_eq!(
            stale_closure_bundle.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut relabeled_closure_bundle = certifications.clone();
        relabeled_closure_bundle
            .volumetric_boundary_closure
            .as_mut()
            .unwrap()
            .operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_closure_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let selected_request = ExactBooleanRequest::new(
            ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
            ValidationPolicy::ALLOW_BOUNDARY,
        );
        let selected_certifications = workspace_certifications(&mut workspace, selected_request);
        selected_certifications
            .validate_for_request(selected_request)
            .unwrap();
        assert!(
            selected_certifications
                .volumetric_boundary_closure
                .is_none()
        );

        let readiness = certifications.winding_readiness.clone();
        assert_eq!(
            readiness.freshness_against_sources_with_boundary_policy(
                &left,
                &right,
                request.validation,
                request.boundary_policy
            ),
            ExactReportFreshness::Current
        );
        let mut stale_readiness_bundle = certifications.clone();
        stale_readiness_bundle.winding_readiness.retained_events += 1;
        assert_eq!(
            stale_readiness_bundle.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        let mut relabeled_readiness_bundle = certifications.clone();
        relabeled_readiness_bundle.winding_readiness.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_readiness_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let planar_report = certifications.planar_arrangement.clone();
        assert_eq!(
            planar_report.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_planar_bundle = certifications.clone();
        stale_planar_bundle.planar_arrangement.retained_events += 1;
        assert_eq!(
            stale_planar_bundle.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut relabeled_planar_bundle = certifications.clone();
        relabeled_planar_bundle.planar_arrangement.operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_planar_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let selected_planar_report = selected_certifications.planar_arrangement;
        assert_eq!(
            selected_planar_report,
            crate::boolean::not_named_planar_arrangement_report(selected_request.operation)
        );
        assert_eq!(selected_planar_report.retained_face_pairs, 0);
        assert_eq!(selected_planar_report.retained_events, 0);

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
        assert_eq!(
            materialize_workspace.evaluations.len(),
            1,
            "first-call materialize should promote the evaluation cache"
        );
        assert_eq!(
            materialize_workspace.evaluations[0].1.result.as_ref(),
            Some(&materialized)
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
        validate_retained_result_for_request(&left, &right, request, &materialized).unwrap();
        let mut locally_invalid_cached_result = materialized.clone();
        locally_invalid_cached_result.graph_had_unknowns =
            !locally_invalid_cached_result.graph_had_unknowns;
        assert!(
            validate_retained_result_for_request(
                &left,
                &right,
                request,
                &locally_invalid_cached_result
            )
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
                validate_retained_result_for_request(&left, &right, request, &stale_gate_report),
                Err(ExactReportValidationError::SourceReplayMismatch)
            );
        }
        assert_eq!(
            exact_report_freshness(validate_retained_result_for_request(
                &left,
                &right,
                request,
                &materialized
            )),
            ExactReportFreshness::Current
        );
        let mut stale_result = materialized.clone();
        stale_result.kind = ExactBooleanResultKind::ArrangementCellComplexMaterialized {
            operation: ExactBooleanOperation::Difference,
        };
        assert!(
            validate_retained_result_for_request(&left, &right, request, &stale_result).is_err()
        );
        assert_ne!(
            exact_report_freshness(validate_retained_result_for_request(
                &left,
                &right,
                request,
                &stale_result
            )),
            ExactReportFreshness::Current
        );

        let evaluation = workspace.evaluate(request).unwrap();
        assert_eq!(
            evaluation.certifications.arrangement_attempt.as_ref(),
            Some(&attempt)
        );
        evaluation.validate().unwrap();
        let first_evaluation = evaluation as *const ExactBooleanEvaluation;
        let second_evaluation =
            workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(first_evaluation, second_evaluation);
        workspace
            .materialize(request)
            .unwrap()
            .validate_against_sources(&left, &right)
            .unwrap();
    }

    #[test]
    fn exact_boolean_workspace_evaluation_retains_certification_set() {
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

        let first = workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        let second = workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(first, second);

        let certifications = workspace_certifications(&mut workspace, request);
        certifications.validate_for_request(request).unwrap();
        certifications
            .validate_against_sources(&left, &right, request)
            .unwrap();
        assert_eq!(
            certifications.freshness_against_sources(&left, &right, request),
            ExactReportFreshness::Current
        );

        let mut stale = certifications;
        stale.refinement.retained_events += 1;
        assert_eq!(
            stale.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_eq!(
            stale.validate_against_sources(&left, &right, request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            stale.freshness_against_sources(&left, &right, request),
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

        assert_eq!(
            attempt.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(workspace.arrangements.len(), 0);

        let evaluation = workspace.evaluate(request).unwrap().clone();
        evaluation.validate().unwrap();
        assert!(evaluation.preflight.is_certified());
        assert!(evaluation.result.is_some());
        assert!(evaluation.certifications.topology_assembly.is_none());
        assert!(evaluation.certifications.region_ownership.is_none());
        assert_eq!(
            evaluation.certifications.arrangement_attempt.as_ref(),
            Some(&attempt)
        );
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
        stale_attempt
            .output_facts
            .as_mut()
            .expect("materialized attempt should retain output facts")
            .face_count += 1;
        stale_attempt.validate().unwrap();
        assert_eq!(
            stale_attempt.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_eq!(workspace.arrangements.len(), 0);
    }

    #[test]
    fn exact_boolean_workspace_evaluation_validates_cached_arrangement_attempt() {
        let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.arrangement_attempts[0].2.operation = ExactBooleanOperation::Difference;

        assert!(
            workspace.evaluate(request).is_err(),
            "cached arrangement attempts must match the evaluated request"
        );

        let mut materialize_workspace = ExactBooleanWorkspace::new(&left, &right);
        materialize_workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        materialize_workspace.arrangement_attempts[0].2.operation =
            ExactBooleanOperation::Difference;

        assert!(
            materialize_workspace.materialize(request).is_err(),
            "materialize must validate cached arrangement attempts through evaluation"
        );
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
        retained.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            retained.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let mut stale = retained.clone();
        stale.preflight.retained_events += 1;
        assert_eq!(
            stale.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        assert_ne!(
            stale.freshness_against_sources(&left, &right),
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
            corrupted.validate_against_sources(&left, &right).is_err(),
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
        assert_eq!(workspace.evaluations.len(), 1);
        assert_eq!(
            workspace.evaluations[0].1.result.as_ref(),
            Some(&materialized)
        );
        let evaluation = workspace.evaluate(request).unwrap().clone();
        assert_eq!(evaluation.result.as_ref(), Some(&materialized));
        assert_eq!(workspace.evaluations.len(), 1);
        assert_eq!(workspace.materializations.len(), 1);
        evaluation.validate().unwrap();

        let mut corrupt_workspace = ExactBooleanWorkspace::new(&left, &right);
        corrupt_workspace.materialize(request).unwrap();
        corrupt_workspace.evaluations.clear();
        corrupt_workspace.materializations[0].1.graph_had_unknowns =
            !corrupt_workspace.materializations[0].1.graph_had_unknowns;
        assert!(
            corrupt_workspace.evaluate(request).is_err(),
            "cached materialization must validate before evaluation reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_materialization_promotes_evaluation_result_cache() {
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

        let evaluation = workspace.evaluate(request).unwrap().clone();
        let evaluated_result = evaluation
            .result
            .expect("certified test request should retain a result");
        assert!(workspace.materializations.is_empty());

        let materialized = workspace.materialize(request).unwrap();
        assert_eq!(materialized, evaluated_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materializations[0].1, evaluated_result);
        assert_eq!(workspace.materialize(request).unwrap(), evaluated_result);
        assert_eq!(workspace.materializations.len(), 1);
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
            validate_retained_result_for_request(&left, &right, request, &relabelled).is_err(),
            "cached result validation must reject relabelled operations"
        );
        assert!(
            workspace.materialize(request).is_err(),
            "cached materialization must match the request operation before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_boundary_touching_canonical_materialization() {
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

        let materialized = workspace.materialize(request).unwrap();
        let expected_result = request.materialize(&left, &right).unwrap();
        assert_eq!(materialized, expected_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materialize(request).unwrap(), materialized);
        assert_eq!(workspace.materializations.len(), 1);

        let mut relabelled = materialized.clone();
        relabelled.kind = ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Difference,
            shortcut: ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        };
        workspace.materializations[0].1 = relabelled;
        assert!(
            workspace.materialize(request).is_err(),
            "cached boundary-touching materialization must match the request operation"
        );
        workspace.materializations[0].1 = materialized.clone();

        workspace.materializations[0].1.graph_had_unknowns =
            !workspace.materializations[0].1.graph_had_unknowns;
        assert!(
            workspace.materialize(request).is_err(),
            "cached boundary-touching materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_closed_no_volume_overlap_canonical_materialization() {
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

        let materialized = workspace.materialize(request).unwrap();
        let expected_result = request.materialize(&left, &right).unwrap();
        let expected_evidence = request
            .evaluate(&left, &right)
            .unwrap()
            .preflight
            .coplanar_volumetric_evidence;
        assert_eq!(materialized, expected_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materialize(request).unwrap(), materialized);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(
            workspace
                .preflight(request)
                .unwrap()
                .coplanar_volumetric_evidence,
            expected_evidence
        );

        workspace.materializations[0].1.graph_had_unknowns =
            !workspace.materializations[0].1.graph_had_unknowns;
        assert!(
            workspace.materialize(request).is_err(),
            "cached no-volume materialization must validate before reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_reuses_adjacent_union_completion_canonical_materialization() {
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

        let materialized = workspace.materialize(request).unwrap();
        let expected_report = crate::boolean::adjacent_union_completion_certification(
            &left,
            &right,
            request.operation,
            None,
        )
        .unwrap()
        .0;
        materialized.validate().unwrap();
        assert!(materialized.matches_request(request));
        assert_eq!(workspace.materialize(request).unwrap(), materialized);
        assert_eq!(
            workspace_certifications(&mut workspace, request).adjacent_union_completion,
            expected_report
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
