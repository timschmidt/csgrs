use super::arrangement3d::ExactArrangement;
use super::boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanCertificationSet, ExactBooleanEvaluation,
    ExactBooleanRequest, arrangement_boolean_attempt_report_from_arrangement,
    arrangement_cell_complex_shortcut_attempt,
    certified_arrangement_cell_complex_preflight_from_retained_attempt,
    materialize_boolean_exact_request_from_retained_graph,
    preflight_boolean_exact_request_from_graph_with_retained_attempt,
    try_materialize_certified_boolean_support_with_artifacts,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{ExactIntersectionGraph, build_intersection_graph};
use super::mesh::ExactMesh;
use super::regularization::ExactRegularizationPolicy;
use super::reports::{
    ExactBooleanPreflight, ExactBooleanResult, ExactBooleanSupport, ExactReportValidationError,
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

    pub(crate) fn validated_graph(&mut self) -> Result<&ExactIntersectionGraph, MeshError> {
        self.ensure_validated_graph()?;
        Ok(self
            .graph
            .as_ref()
            .expect("validated graph cache was just populated"))
    }

    fn ensure_validated_graph(&mut self) -> Result<(), MeshError> {
        if self.graph.is_none() {
            self.graph = Some(build_intersection_graph(self.left, self.right)?);
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph was just initialized");
        graph
            .validate_against_meshes(self.left, self.right)
            .map_err(|error| {
                MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    format!("exact boolean workspace graph failed validation: {error:?}"),
                ))
            })?;
        Ok(())
    }

    pub(crate) fn into_validated_graph(mut self) -> Result<ExactIntersectionGraph, MeshError> {
        self.validated_graph()?;
        Ok(self
            .graph
            .take()
            .expect("validated graph cache was just populated"))
    }

    pub(crate) fn into_arrangement_attempt(
        mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactArrangementBooleanAttempt, MeshError> {
        self.arrangement_attempt(request, policy)?;
        let index = cached_by_request_and_policy_index(&self.arrangement_attempts, request, policy)
            .expect("arrangement attempt cache was just populated");
        Ok(self.arrangement_attempts.swap_remove(index).2)
    }

    pub(crate) fn into_evaluation(
        mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanEvaluation, MeshError> {
        self.evaluate(request)?;
        let index = cached_by_request_index(&self.evaluations, request)
            .expect("evaluation cache was just populated");
        Ok(self.evaluations.swap_remove(index).1)
    }

    fn regularized_solid_arrangement(&self) -> Option<&ExactArrangement> {
        cached_by_policy_index(
            &self.arrangements,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .map(|index| &self.arrangements[index].1)
    }

    fn validated_regularized_solid_arrangement_attempt(
        &self,
        request: ExactBooleanRequest,
    ) -> Result<Option<&ExactArrangementBooleanAttempt>, MeshError> {
        let Some(index) = cached_by_request_and_policy_index(
            &self.arrangement_attempts,
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        ) else {
            return Ok(None);
        };
        self.arrangement_attempts[index]
            .2
            .validate_against_sources_for_request(self.left, self.right, request)
            .map_err(workspace_report_validation_error)?;
        Ok(Some(&self.arrangement_attempts[index].2))
    }

    /// Returns the exact arrangement for `policy`, building it once per policy.
    pub(crate) fn arrangement(
        &mut self,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangement, MeshError> {
        if let Some(index) = cached_by_policy_index(&self.arrangements, policy) {
            self.arrangements[index]
                .1
                .validate_against_sources_with_policy(self.left, self.right, policy)
                .map_err(|blocker| {
                    MeshError::one(MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::UnsupportedExactOperation,
                        format!(
                            "exact boolean workspace arrangement cache failed replay: {blocker:?}"
                        ),
                    ))
                })?;
            return Ok(&self.arrangements[index].1);
        }

        let graph = self.validated_graph()?.clone();
        let arrangement = ExactArrangement::from_intersection_graph_with_policy(
            graph, self.left, self.right, policy,
        )?;
        arrangement.validate().map_err(|blocker| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact boolean workspace arrangement report failed: {blocker:?}"),
            ))
        })?;
        self.arrangements.push((policy, arrangement));
        let index = self.arrangements.len() - 1;
        debug_assert_eq!(self.arrangements[index].0, policy);
        Ok(&self.arrangements[index].1)
    }

    /// Returns the arrangement/cell-complex attempt report for `request` and
    /// `policy`, reusing the cached arrangement for that policy.
    pub(crate) fn arrangement_attempt(
        &mut self,
        request: ExactBooleanRequest,
        policy: ExactRegularizationPolicy,
    ) -> Result<&ExactArrangementBooleanAttempt, MeshError> {
        if let Some(index) =
            cached_by_request_and_policy_index(&self.arrangement_attempts, request, policy)
        {
            self.arrangement_attempts[index]
                .2
                .validate_against_sources_for_request(self.left, self.right, request)
                .map_err(workspace_report_validation_error)?;
            return Ok(&self.arrangement_attempts[index].2);
        }

        let left = self.left;
        let right = self.right;
        let attempt = match self.arrangement(policy) {
            Ok(arrangement) => {
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
                    arrangement_cell_complex_shortcut_attempt(left, right, request, policy)?
                        .unwrap_or(attempt)
                }
            }
            Err(error) => {
                if let Some(attempt) =
                    arrangement_cell_complex_shortcut_attempt(left, right, request, policy)?
                {
                    attempt
                } else {
                    return Err(error);
                }
            }
        };
        attempt
            .validate_for_request_policy(request, policy)
            .map_err(workspace_report_validation_error)?;
        self.arrangement_attempts.push((request, policy, attempt));
        let index = self.arrangement_attempts.len() - 1;
        debug_assert_eq!(self.arrangement_attempts[index].0, request);
        debug_assert_eq!(self.arrangement_attempts[index].1, policy);
        Ok(&self.arrangement_attempts[index].2)
    }

    /// Derive preflight for `request` from the retained graph.
    pub(crate) fn preflight(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanPreflight, MeshError> {
        let left = self.left;
        let right = self.right;
        if cached_by_request_and_policy_index(
            &self.arrangement_attempts,
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .is_none()
            && !matches!(
                request.operation,
                super::boolean::ExactBooleanOperation::SelectedRegions(_)
            )
        {
            let _ = self.arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID);
        }
        self.ensure_validated_graph()?;
        let retained_attempt = self.validated_regularized_solid_arrangement_attempt(request)?;
        let graph = self
            .graph
            .as_ref()
            .expect("validated graph cache was just populated");
        let graph_preflight = preflight_boolean_exact_request_from_graph_with_retained_attempt(
            graph,
            left,
            right,
            request,
            retained_attempt,
        )?;
        if graph_preflight.operation != request.operation {
            return Err(workspace_report_validation_error(
                ExactReportValidationError::StatusEvidenceMismatch,
            ));
        }
        graph_preflight
            .validate()
            .map_err(workspace_report_validation_error)?;
        if matches!(
            graph_preflight.support,
            ExactBooleanSupport::CertifiedEmptyOperand
                | ExactBooleanSupport::CertifiedBoundsDisjoint
                | ExactBooleanSupport::CertifiedIdentical
                | ExactBooleanSupport::CertifiedSameSurface
                | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
                | ExactBooleanSupport::CertifiedConvexSeparated
        ) {
            return Ok(graph_preflight);
        }
        if let Some(attempt) = retained_attempt
            && let Some(preflight) =
                certified_arrangement_cell_complex_preflight_from_retained_attempt(
                    graph,
                    left,
                    right,
                    request,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                    attempt,
                )?
        {
            preflight
                .validate()
                .map_err(workspace_report_validation_error)?;
            return Ok(preflight);
        }
        Ok(graph_preflight)
    }

    /// Returns an exact boolean evaluation for `request`, building it once per
    /// request.
    pub fn evaluate(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanEvaluation, MeshError> {
        if let Some(index) = cached_by_request_index(&self.evaluations, request) {
            self.evaluations[index]
                .1
                .validate_against_sources(self.left, self.right)
                .map_err(workspace_report_validation_error)?;
            return Ok(&self.evaluations[index].1);
        }

        let preflight = self.preflight(request)?;
        if preflight.support == ExactBooleanSupport::CertifiedArrangementCellComplex {
            self.arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)?;
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let regularized_arrangement = self.regularized_solid_arrangement();
        let regularized_attempt = self.validated_regularized_solid_arrangement_attempt(request)?;
        let certifications = ExactBooleanCertificationSet::from_graph_and_regularized_arrangement(
            graph,
            self.left,
            self.right,
            request,
            regularized_arrangement,
            regularized_attempt,
        )?;
        let result = if preflight.is_certified() {
            if let Some(index) = cached_by_request_index(&self.materializations, request) {
                self.validate_materialized_result_for_request(
                    request,
                    &self.materializations[index].1,
                )?;
                Some(self.materializations[index].1.clone())
            } else if matches!(preflight.support, ExactBooleanSupport::SelectedRegionPolicy) {
                self.try_materialize_certified_support(
                    request,
                    preflight.support,
                    regularized_arrangement,
                    regularized_attempt,
                )
                .ok()
                .flatten()
            } else {
                self.try_materialize_certified_support(
                    request,
                    preflight.support,
                    regularized_arrangement,
                    regularized_attempt,
                )?
            }
        } else {
            None
        };
        let evaluation =
            ExactBooleanEvaluation::from_parts(request, preflight, certifications, result)
                .map_err(workspace_report_validation_error)?;
        self.evaluations.push((request, evaluation));
        let index = self.evaluations.len() - 1;
        debug_assert_eq!(self.evaluations[index].0, request);
        Ok(&self.evaluations[index].1)
    }

    fn materialize_uncached(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<ExactBooleanResult, MeshError> {
        let evaluation = self.evaluate(request)?;
        let certified_support = evaluation
            .preflight()
            .is_certified()
            .then_some(evaluation.preflight().support);
        let retained_result = evaluation.materialized_result().cloned();
        if let Some(support) = certified_support {
            if let Some(result) = retained_result {
                return Ok(result);
            }
            let regularized_arrangement = self.regularized_solid_arrangement();
            let regularized_attempt =
                self.validated_regularized_solid_arrangement_attempt(request)?;
            let result = self
                .try_materialize_certified_support(
                    request,
                    support,
                    regularized_arrangement,
                    regularized_attempt,
                )?
                .ok_or_else(|| {
                    workspace_report_validation_error(
                        ExactReportValidationError::StatusEvidenceMismatch,
                    )
                })?;
            return Ok(result);
        }
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        let result = materialize_boolean_exact_request_from_retained_graph(
            graph, self.left, self.right, request,
        )?;
        Ok(result)
    }

    /// Materialize an exact boolean result and return the cached retained value.
    ///
    /// This is the public materialization path so callers consume the retained
    /// replay cache tied to this workspace session.
    pub fn materialize_ref(
        &mut self,
        request: ExactBooleanRequest,
    ) -> Result<&ExactBooleanResult, MeshError> {
        if let Some(index) = cached_by_request_index(&self.materializations, request) {
            self.validate_materialization_and_sync_evaluation_cache(request, index)?;
            return Ok(&self.materializations[index].1);
        }

        let result = self.materialize_uncached(request)?;
        self.materializations.push((request, result));
        let index = self.materializations.len() - 1;
        self.validate_materialization_and_sync_evaluation_cache(request, index)?;
        Ok(&self.materializations[index].1)
    }

    fn validate_materialized_result_for_request(
        &self,
        request: ExactBooleanRequest,
        result: &ExactBooleanResult,
    ) -> Result<(), MeshError> {
        let retained_attempt = self.validated_regularized_solid_arrangement_attempt(request)?;
        result
            .validate_request_against_sources_with_retained_attempt(
                self.left,
                self.right,
                request,
                retained_attempt,
            )
            .map_err(workspace_report_validation_error)
    }

    fn try_materialize_certified_support(
        &self,
        request: ExactBooleanRequest,
        support: ExactBooleanSupport,
        regularized_arrangement: Option<&ExactArrangement>,
        regularized_attempt: Option<&ExactArrangementBooleanAttempt>,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        let graph = self
            .graph
            .as_ref()
            .expect("intersection graph cache was just populated");
        try_materialize_certified_boolean_support_with_artifacts(
            self.left,
            self.right,
            request,
            support,
            Some(graph),
            regularized_arrangement,
            regularized_attempt,
        )
    }

    fn validate_materialization_and_sync_evaluation_cache(
        &mut self,
        request: ExactBooleanRequest,
        materialization_index: usize,
    ) -> Result<(), MeshError> {
        debug_assert_eq!(self.materializations[materialization_index].0, request);
        self.validate_materialized_result_for_request(
            request,
            &self.materializations[materialization_index].1,
        )?;
        if cached_by_request_index(&self.evaluations, request).is_none() {
            self.evaluate(request)?;
        }
        let result = &self.materializations[materialization_index].1;
        let evaluation_index =
            cached_by_request_index(&self.evaluations, request).ok_or_else(|| {
                workspace_report_validation_error(
                    ExactReportValidationError::StatusEvidenceMismatch,
                )
            })?;
        self.evaluations[evaluation_index]
            .1
            .retain_materialized_result(result)
            .map_err(workspace_report_validation_error)
    }
}

fn cached_by_policy_index<T>(
    cache: &[(ExactRegularizationPolicy, T)],
    policy: ExactRegularizationPolicy,
) -> Option<usize> {
    cache.iter().position(|(cached, _)| *cached == policy)
}

fn cached_by_request_index<T>(
    cache: &[(ExactBooleanRequest, T)],
    request: ExactBooleanRequest,
) -> Option<usize> {
    cache.iter().position(|(cached, _)| *cached == request)
}

fn cached_by_request_and_policy_index<T>(
    cache: &[(ExactBooleanRequest, ExactRegularizationPolicy, T)],
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> Option<usize> {
    cache.iter().position(|(cached_request, cached_policy, _)| {
        *cached_request == request && *cached_policy == policy
    })
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
    use crate::reports::ExactReportFreshness;
    use crate::reports::ExactReportValidationError;
    use crate::reports::{ExactBooleanResultKind, ExactBooleanShortcutKind};
    use crate::validation::ValidationPolicy;
    use crate::{ExactBoundaryBooleanPolicy, Triangle};

    #[test]
    fn exact_boolean_workspace_reuses_graph_arrangement_preflight_and_evaluation() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);

        let first_graph = workspace.validated_graph().unwrap() as *const ExactIntersectionGraph;
        let second_graph = workspace.validated_graph().unwrap() as *const ExactIntersectionGraph;
        assert_eq!(first_graph, second_graph);

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
        let mut stale_topology_report = topology_report.clone();
        stale_topology_report.graph_events += 1;
        stale_topology_report.validate().unwrap();

        let ownership_report = attempt_with_reports
            .region_ownership_report
            .clone()
            .expect("arrangement attempt should retain ownership evidence");
        ownership_report.validate().unwrap();
        let mut stale_ownership_report = ownership_report.clone();
        stale_ownership_report.face_cell_boundary_nodes += 3;
        stale_ownership_report.face_cell_boundary_points += 3;
        stale_ownership_report.validate().unwrap();

        let first_attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangementBooleanAttempt;
        let second_attempt = workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap() as *const ExactArrangementBooleanAttempt;
        assert_eq!(first_attempt, second_attempt);
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

        let selected = attempt_selected;
        assert!(selected.topology_assembly_report.is_some());
        assert!(selected.region_ownership_report.is_some());
        selected.validate().unwrap();
        let mismatched_request =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED);
        assert_ne!(
            selected.operation, mismatched_request.operation,
            "selected artifact should carry the request operation it proves"
        );
        let mut stale_selected = selected.clone();
        stale_selected.selected_faces.pop();
        assert!(stale_selected.validate().is_err());
        stale_selected
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;

        let simplified = attempt_simplified;
        assert!(simplified.topology_assembly_report.is_some());
        assert!(simplified.region_ownership_report.is_some());
        simplified.validate().unwrap();
        assert_ne!(
            simplified.operation, mismatched_request.operation,
            "simplified artifact should carry the request operation it proves"
        );
        let mut stale_simplified = simplified.clone();
        stale_simplified.duplicate_cells_removed += 1;
        assert!(stale_simplified.validate().is_err());
        stale_simplified
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .graph_events += 1;

        let first_preflight = workspace.preflight(request).unwrap();
        let second_preflight = workspace.preflight(request).unwrap();
        assert_eq!(first_preflight, second_preflight);
        let mut replay_workspace = ExactBooleanWorkspace::new(&left, &right);
        assert_eq!(
            first_preflight,
            replay_workspace.preflight(request).unwrap()
        );
        let preflight = first_preflight;
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();
        assert_eq!(
            preflight.freshness_against_sources_for_request(&left, &right, request),
            ExactReportFreshness::Current
        );
        let mut stale_preflight = preflight.clone();
        stale_preflight.retained_events += 1;
        assert_eq!(
            stale_preflight.validate_against_sources_for_request(&left, &right, request),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert_ne!(
            stale_preflight.freshness_against_sources_for_request(&left, &right, request),
            ExactReportFreshness::Current
        );

        let evaluation = workspace.evaluate(request).unwrap();
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert_eq!(evaluation.retained_arrangement_attempt(), Some(&attempt));

        let refinement_report = evaluation.certifications().refinement().clone();
        assert_eq!(
            refinement_report.freshness_against_sources_for_request(&left, &right, request),
            ExactReportFreshness::Current
        );
        let certifications = evaluation.certifications().clone();
        let mut stale_refinement_bundle = certifications.clone();
        stale_refinement_bundle.refinement_mut().retained_events += 1;
        assert_eq!(
            stale_refinement_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut relabeled_refinement_bundle = certifications.clone();
        relabeled_refinement_bundle.refinement_mut().operation = ExactBooleanOperation::Difference;
        assert_eq!(
            relabeled_refinement_bundle.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let evaluation = workspace.evaluate(request).unwrap();
        assert_eq!(evaluation.retained_arrangement_attempt(), Some(&attempt));
        evaluation.validate().unwrap();
        let first_evaluation = evaluation as *const ExactBooleanEvaluation;
        let second_evaluation =
            workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(first_evaluation, second_evaluation);
        workspace
            .materialize_ref(request)
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
        workspace.validated_graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();
        assert_eq!(
            workspace.arrangement_attempts.len(),
            1,
            "preflight should retain the arrangement attempt it uses for certification"
        );

        let first = workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(
            workspace.arrangement_attempts.len(),
            1,
            "evaluation should promote cell-complex certification through the retained attempt cache"
        );
        assert_eq!(
            workspace.evaluations[0].1.retained_arrangement_attempt(),
            Some(&workspace.arrangement_attempts[0].2)
        );
        let second = workspace.evaluate(request).unwrap() as *const ExactBooleanEvaluation;
        assert_eq!(first, second);

        let evaluation = workspace.evaluate(request).unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        let certifications = evaluation.certifications().clone();
        certifications.validate_for_request(request).unwrap();

        let mut stale = certifications;
        stale.refinement_mut().retained_events += 1;
        assert_eq!(
            stale.validate_for_request(request),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn exact_boolean_workspace_evaluation_validates_cached_arrangement_attempt() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
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
            materialize_workspace.materialize_ref(request).is_err(),
            "materialize must validate cached arrangement attempts through evaluation"
        );

        let mut stale_workspace = ExactBooleanWorkspace::new(&left, &right);
        stale_workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        stale_workspace.arrangement_attempts[0].2.output_triangles += 1;
        stale_workspace.arrangement_attempts[0]
            .2
            .output_facts
            .as_mut()
            .expect("materialized attempt should retain output facts")
            .face_count += 1;
        stale_workspace.arrangement_attempts[0]
            .2
            .validate()
            .unwrap();
        assert_eq!(
            stale_workspace.arrangement_attempts[0]
                .2
                .validate_against_sources_for_request(&left, &right, request),
            Err(ExactReportValidationError::SourceReplayMismatch)
        );
        assert!(
            stale_workspace.evaluate(request).is_err(),
            "cached arrangement attempts must replay against sources before evaluation reuse"
        );

        let mut stale_materialize_workspace = ExactBooleanWorkspace::new(&left, &right);
        stale_materialize_workspace
            .arrangement_attempt(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        stale_materialize_workspace.arrangement_attempts[0]
            .2
            .output_triangles += 1;
        stale_materialize_workspace.arrangement_attempts[0]
            .2
            .output_facts
            .as_mut()
            .expect("materialized attempt should retain output facts")
            .face_count += 1;
        stale_materialize_workspace.arrangement_attempts[0]
            .2
            .validate()
            .unwrap();
        assert!(
            stale_materialize_workspace
                .materialize_ref(request)
                .is_err(),
            "cached arrangement attempts must replay against sources before materialization reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_validates_cached_evaluation_locally() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.validated_graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let retained = workspace.evaluate(request).unwrap();
        retained.validate_against_sources(&left, &right).unwrap();

        let mut stale = retained.clone();
        stale.preflight_mut().retained_events += 1;
        assert_eq!(
            stale.validate_against_sources(&left, &right),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut corrupt_proof_workspace = ExactBooleanWorkspace::new(&left, &right);
        corrupt_proof_workspace.evaluate(request).unwrap();
        corrupt_proof_workspace.evaluations[0]
            .1
            .preflight_mut()
            .retained_events += 1;
        assert!(
            corrupt_proof_workspace.materialize_ref(request).is_err(),
            "cached evaluation proof must validate before materialization reuse"
        );

        let cached_result = workspace.evaluations[0]
            .1
            .materialized_result_mut()
            .expect("certified test request should retain a result");
        cached_result.set_graph_had_unknowns(!cached_result.graph_had_unknowns());
        let corrupted = workspace.evaluations[0].1.clone();
        assert!(
            corrupted.validate_against_sources(&left, &right).is_err(),
            "cached evaluation validation must still enforce local invariants"
        );
        assert!(
            workspace.materialize_ref(request).is_err(),
            "cached evaluation results must validate before materialization reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_evaluation_reuses_materialization_cache() {
        let left = tetra([0, 0, 0]);
        let right = tetra([4, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.validated_graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.evaluations.len(), 1);
        assert_eq!(
            workspace.evaluations[0].1.materialized_result(),
            Some(&materialized)
        );
        {
            let evaluation = workspace.evaluate(request).unwrap();
            assert_eq!(evaluation.materialized_result(), Some(&materialized));
            evaluation.validate().unwrap();
        }
        assert_eq!(workspace.evaluations.len(), 1);
        assert_eq!(workspace.materializations.len(), 1);

        let mut corrupt_workspace = ExactBooleanWorkspace::new(&left, &right);
        corrupt_workspace.materialize_ref(request).unwrap();
        corrupt_workspace.evaluations.clear();
        let graph_had_unknowns = corrupt_workspace.materializations[0].1.graph_had_unknowns();
        corrupt_workspace.materializations[0]
            .1
            .set_graph_had_unknowns(!graph_had_unknowns);
        assert!(
            corrupt_workspace.evaluate(request).is_err(),
            "cached materialization must validate before evaluation reuse"
        );
    }

    #[test]
    fn exact_boolean_workspace_materialization_promotes_evaluation_result_cache() {
        let left = tetra([0, 0, 0]);
        let right = tetra([4, 0, 0]);
        let request =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        workspace.validated_graph().unwrap();
        workspace
            .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        workspace.preflight(request).unwrap();

        let evaluated_result = {
            let evaluation = workspace.evaluate(request).unwrap();
            evaluation.validate().unwrap();
            evaluation
                .materialized_result()
                .cloned()
                .expect("certified test request should retain a result")
        };
        assert!(workspace.materializations.is_empty());

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
        assert_eq!(materialized, evaluated_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materializations[0].1, evaluated_result);
        assert_eq!(
            workspace.materialize_ref(request).unwrap(),
            &evaluated_result
        );
        assert_eq!(workspace.materializations.len(), 1);
        let borrowed = workspace.materialize_ref(request).unwrap() as *const ExactBooleanResult;
        let cached = &workspace.materializations[0].1 as *const ExactBooleanResult;
        assert_eq!(borrowed, cached);
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

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
        assert!(materialized.is_boundary_policy_shortcut_for(ExactBooleanOperation::Union));
        workspace.materializations[0].1.replace_kind(
            ExactBooleanResultKind::BoundaryPolicyShortcut {
                operation: ExactBooleanOperation::Difference,
            },
        );
        let relabelled = workspace.materializations[0].1.clone();
        assert!(
            relabelled
                .validate_request_against_sources_with_retained_attempt(
                    &left, &right, request, None
                )
                .is_err(),
            "cached result validation must reject relabelled operations"
        );
        assert!(
            workspace.materialize_ref(request).is_err(),
            "cached materialization must match the request operation before reuse"
        );
        assert!(
            workspace.materialize_ref(request).is_err(),
            "borrowed materialization must validate cached results before reuse"
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

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
        let expected_result = ExactBooleanWorkspace::new(&left, &right)
            .materialize_ref(request)
            .cloned()
            .unwrap();
        assert_eq!(materialized, expected_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materialize_ref(request).unwrap(), &materialized);
        assert_eq!(workspace.materializations.len(), 1);

        let mut relabelled = materialized.clone();
        relabelled.replace_kind(ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Difference,
            shortcut: ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        });
        workspace.materializations[0].1 = relabelled;
        assert!(
            workspace.materialize_ref(request).is_err(),
            "cached boundary-touching materialization must match the request operation"
        );
        workspace.materializations[0].1 = materialized.clone();

        let graph_had_unknowns = workspace.materializations[0].1.graph_had_unknowns();
        workspace.materializations[0]
            .1
            .set_graph_had_unknowns(!graph_had_unknowns);
        assert!(
            workspace.materialize_ref(request).is_err(),
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

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
        let expected_result = ExactBooleanWorkspace::new(&left, &right)
            .materialize_ref(request)
            .cloned()
            .unwrap();
        let expected_evidence = ExactBooleanWorkspace::new(&left, &right)
            .evaluate(request)
            .unwrap()
            .preflight()
            .clone()
            .coplanar_volumetric_evidence;
        assert_eq!(materialized, expected_result);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(workspace.materialize_ref(request).unwrap(), &materialized);
        assert_eq!(workspace.materializations.len(), 1);
        assert_eq!(
            workspace
                .preflight(request)
                .unwrap()
                .coplanar_volumetric_evidence,
            expected_evidence
        );

        let graph_had_unknowns = workspace.materializations[0].1.graph_had_unknowns();
        workspace.materializations[0]
            .1
            .set_graph_had_unknowns(!graph_had_unknowns);
        assert!(
            workspace.materialize_ref(request).is_err(),
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

        let materialized = workspace.materialize_ref(request).cloned().unwrap();
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
        assert_eq!(workspace.materialize_ref(request).unwrap(), &materialized);
        let evaluation = workspace.evaluate(request).unwrap();
        evaluation.validate().unwrap();
        assert_eq!(
            evaluation
                .certifications()
                .adjacent_union_completion()
                .clone(),
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
