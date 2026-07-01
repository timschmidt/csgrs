use super::*;

/// Complete exact boolean evaluation outcome used by replay/audit tests.
///
/// `result` is present only when the request materialized under retained exact
/// evidence. When it is absent, `preflight` and `certifications` retain the
/// blocker/provenance facts instead of collapsing the request to an
/// approximate or prose-only error.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactBooleanEvaluation {
    pub(super) request: ExactBooleanRequest,
    pub(super) preflight: ExactBooleanPreflight,
    pub(super) certifications: ExactBooleanCertificationSet,
    pub(super) result: Option<ExactBooleanResult>,
}

pub(super) fn exact_boolean_evaluation_for_replay_result_with_materialization(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    materialize_result: bool,
) -> Result<ExactBooleanEvaluation, ExactMeshError> {
    left.validate_retained_bounds()?;
    right.validate_retained_bounds()?;
    let prepared_pair = left.view().prepare_broad_phase_pair(right.view())?;
    let shortcut_facts = prepared_pair.prepare_arrangement_cell_complex_shortcut_facts()?;
    let graph = prepared_pair.validated_intersection_graph()?;
    let mut regularized_arrangement = None;
    let mut regularized_attempt = None;
    let mut preflight = exact_boolean_replay_preflight(
        left,
        right,
        request,
        graph.as_ref(),
        &shortcut_facts,
        regularized_attempt.as_ref(),
    )?;
    let certified_by_coplanar_boundary_closure = preflight.support
        == ExactBooleanSupport::CertifiedArrangementCellComplex
        && request.validation == ExactMeshValidationPolicy::CLOSED
        && preflight.coplanar_volumetric_evidence.as_ref().is_some();
    let certified_by_orthogonal_cell_materialization = preflight.support
        == ExactBooleanSupport::CertifiedArrangementCellComplex
        && orthogonal_solid_cell_materializes_for_preflight(left, right, request.operation)?;
    let should_replay_arrangement = !certified_by_coplanar_boundary_closure
        && !certified_by_orthogonal_cell_materialization
        && matches!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
        )
        || (!certified_by_coplanar_boundary_closure
            && !certified_by_orthogonal_cell_materialization
            && !graph.face_pairs.is_empty()
            && matches!(
                preflight.support,
                ExactBooleanSupport::CertifiedConvexUnion
                    | ExactBooleanSupport::CertifiedConvexIntersection
                    | ExactBooleanSupport::CertifiedConvexDifference
            ));
    if should_replay_arrangement {
        replay_regularized_arrangement_attempt(
            left,
            right,
            request,
            graph.as_ref(),
            &shortcut_facts,
            &mut regularized_arrangement,
            &mut regularized_attempt,
        )?;
        if regularized_attempt.is_some() {
            preflight = exact_boolean_replay_preflight(
                left,
                right,
                request,
                graph.as_ref(),
                &shortcut_facts,
                regularized_attempt.as_ref(),
            )?;
        }
    }
    let certifications = certification_set_from_graph_and_regularized_arrangement(
        graph.as_ref(),
        left,
        right,
        request,
        regularized_arrangement.as_ref(),
        regularized_attempt.as_ref(),
    )?;
    let result = if materialize_result
        && preflight.support.is_certified()
        && matches!(&preflight.blocker, None)
    {
        if matches!(preflight.support, ExactBooleanSupport::SelectedRegionPolicy) {
            try_materialize_certified_boolean_support_with_artifacts(
                left,
                right,
                request,
                preflight.support,
                Some(graph.as_ref()),
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
                preflight.support,
                Some(graph.as_ref()),
                regularized_arrangement.as_ref(),
                regularized_attempt.as_ref(),
                &shortcut_facts,
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
    evaluation
        .validate_with_missing_result_policy(!materialize_result)
        .map_err(|error| {
            retained_evidence_validation_error(
                RETAINED_EVIDENCE_REPLAY_CONTEXT,
                error,
                ExactMeshBlockerKind::StaleFactReplay,
            )
        })?;
    Ok(evaluation)
}

pub(super) fn replay_regularized_arrangement_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    graph: &ExactIntersectionGraph,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
    retained_arrangement: &mut Option<ExactArrangement3d>,
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
            let attempt = match run_arrangement_cell_complex_attempt_from_arrangement(
                arrangement,
                left,
                right,
                request,
                policy,
                true,
            )? {
                ArrangementCellComplexOutcome::Materialized(_, attempt)
                | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
            };
            arrangement_cell_complex_attempt_or_shortcut(
                left,
                right,
                request,
                policy,
                shortcut_facts,
                attempt,
            )?
        }
        None => match ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
            graph.clone(),
            left,
            right,
            policy,
        ) {
            Ok(arrangement) => {
                arrangement.validate().map_err(|blocker| {
                    boolean_validation_error(
                        ExactMeshBlockerKind::ExactConstructionFailure,
                        "exact boolean arrangement report failed",
                        blocker,
                    )
                })?;
                let attempt = match run_arrangement_cell_complex_attempt_from_arrangement(
                    &arrangement,
                    left,
                    right,
                    request,
                    policy,
                    true,
                )? {
                    ArrangementCellComplexOutcome::Materialized(_, attempt)
                    | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
                };
                *retained_arrangement = Some(arrangement);
                arrangement_cell_complex_attempt_or_shortcut(
                    left,
                    right,
                    request,
                    policy,
                    shortcut_facts,
                    attempt,
                )?
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
    let graph_preflight_has_source_arrangement_shortcut =
        shortcut_facts.materializes_operation(request.operation);
    let graph_preflight_has_certified_axis_aligned_box_pair = shortcut_facts.axis_aligned_box_pair;
    let graph_preflight = preflight_boolean_exact_request_from_graph_with_retained_attempt(
        graph,
        left,
        right,
        request,
        retained_attempt,
        shortcut_facts,
    )?;
    if graph_preflight.operation != request.operation {
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
        graph_preflight.support,
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
        graph_preflight.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
    ) && !graph_preflight_has_source_arrangement_shortcut
        && !graph_preflight_has_certified_axis_aligned_box_pair)
    {
        return Ok(graph_preflight);
    }
    if ((request.validation != ExactMeshValidationPolicy::ALLOW_BOUNDARY)
        || graph_preflight_has_source_arrangement_shortcut
        || graph_preflight_has_certified_axis_aligned_box_pair)
        && let Some(attempt) = retained_attempt
        && let Ok(Some(preflight)) =
            certified_arrangement_cell_complex_preflight_from_retained_attempt(
                graph, left, right, request, attempt,
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

const RETAINED_EVIDENCE_REPLAY_CONTEXT: &str =
    "exact boolean retained evidence failed replay validation";
