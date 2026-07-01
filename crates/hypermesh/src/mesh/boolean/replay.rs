use super::evidence::{
    ExactBooleanCertificationSet, ExactConvexBooleanCapabilityFacts, ExactRefinementReport,
    ExactRefinementStatus, ExactRegularizedSolidBooleanFacts, ExactTrivialBooleanFacts,
    identical_mesh_report_from_sources, same_surface_report_from_sources,
};
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

fn certification_set_from_graph_and_regularized_arrangement(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_regularized_arrangement: Option<&ExactArrangement3d>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<ExactBooleanCertificationSet, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    if let Some(attempt) = retained_arrangement_attempt {
        attempt
            .validate_for_request_policy(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .map_err(|error| {
                retained_evidence_validation_error(
                    "retained arrangement attempt failed validation",
                    error,
                    ExactMeshBlockerKind::ExactConstructionFailure,
                )
            })?;
    }
    let left_empty = left.facts().mesh.face_count == 0;
    let right_empty = right.facts().mesh.face_count == 0;
    let trivial = ExactTrivialBooleanFacts {
        left_empty,
        right_empty,
        bounds_disjoint: !left_empty
            && !right_empty
            && meshes_are_certified_bounds_disjoint(left, right),
    };
    let regularized_solid = ExactRegularizedSolidBooleanFacts {
        left_closed_solid: !left_empty && left.facts().mesh.closed_manifold,
        right_closed_solid: !right_empty && right.facts().mesh.closed_manifold,
        left_open_surface: mesh_is_open_surface(left),
        right_open_surface: mesh_is_open_surface(right),
    };
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    let graph_had_unknowns = graph.has_unknowns();
    let needs_refinement = graph_had_unknowns || counts.construction_failed_events > 0;
    let refinement = ExactRefinementReport {
        operation: request.operation,
        status: if needs_refinement {
            ExactRefinementStatus::Required
        } else {
            ExactRefinementStatus::NotRequired
        },
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: needs_refinement.then(|| counts.into_blocker(ExactBooleanBlockerKind::Refinement)),
    };
    let boundary_touching =
        boundary_touching_report_from_graph(graph, left, right).unwrap_or_else(|_| {
            let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
            let blocker_kind = counts.inferred_kind();
            ExactBoundaryTouchingReport {
                status: ExactBoundaryTouchingStatus::NotBoundaryOnly,
                graph_had_unknowns: graph.has_unknowns(),
                retained_face_pairs: graph.face_pairs.len(),
                retained_events: graph.event_count(),
                blocker: counts.into_blocker(blocker_kind),
            }
        });
    let open_surface_disjoint = open_surface_disjoint_report_from_graph(graph, left, right);
    let adjacent_union_completion = adjacent_union_completion_certification_from_graph(
        graph,
        left,
        right,
        request.operation,
        None,
    )?
    .0;
    let adjacent_union_completion_certified = matches!(
        adjacent_union_completion.status,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
            | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    );
    let identical = identical_mesh_report_from_sources(left, right);
    let same_surface = same_surface_report_from_sources(left, right);
    let closed_winding_left_in_right =
        classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    let closed_winding_right_in_left =
        classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    let convex_left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    let convex_right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    let convex_capabilities = ExactConvexBooleanCapabilityFacts {
        can_union: certified_convex_operation_shortcut_support(
            left,
            right,
            ExactBooleanOperation::Union,
        )
        .is_some(),
        can_intersection: certified_convex_operation_shortcut_support(
            left,
            right,
            ExactBooleanOperation::Intersection,
        )
        .is_some(),
        can_difference: certified_convex_operation_shortcut_support(
            left,
            right,
            ExactBooleanOperation::Difference,
        )
        .is_some(),
    };
    let arrangement_cell_complex_shortcuts =
        ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    let reject_boundary_evidence_request =
        request.validation == ExactMeshValidationPolicy::ALLOW_BOUNDARY;
    let planar_arrangement =
        if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
            planar_arrangement_report(
                request.operation,
                ExactPlanarArrangementStatus::NotNamedOperation,
                false,
                0,
                0,
                ExactBooleanBlocker::default(),
                None,
            )
        } else {
            let mut arrangement_cell_complex_preflight = None;
            planar_arrangement_report_from_graph_with_cell_complex_cache(
                graph,
                left,
                right,
                request.operation,
                &mut arrangement_cell_complex_preflight,
                Some(request),
                retained_arrangement_attempt,
            )
            .unwrap_or_else(|_| {
                planar_arrangement_report(
                    request.operation,
                    ExactPlanarArrangementStatus::NoPositiveOverlap,
                    graph.has_unknowns(),
                    graph.face_pairs.len(),
                    graph.event_count(),
                    ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding),
                    None,
                )
            })
        };
    let volumetric_boundary_closure =
        if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_))
            || reject_boundary_evidence_request
        {
            None
        } else if adjacent_union_completion_certified {
            Some(no_materialized_boundary_output_report(request.operation))
        } else {
            let report = volumetric_boundary_closure_report_from_graph(
                graph,
                left,
                right,
                request.operation,
            )?;
            validate_volumetric_boundary_closure_report(&report)?;
            if request.validation == ExactMeshValidationPolicy::CLOSED
                && !matches!(
                    report.status,
                    ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
                )
            {
                None
            } else {
                Some(report)
            }
        };
    let arrangement_attempt = if adjacent_union_completion_certified {
        None
    } else {
        let retained_arrangement_attempt_materializes_output = retained_arrangement_attempt
            .is_some_and(
                ExactArrangementBooleanAttempt::materialized_arrangement_cell_complex_output,
            );
        if let Some(attempt) = retained_arrangement_attempt
            && retained_arrangement_attempt_materializes_output
        {
            Some(attempt.clone())
        } else {
            let retained_arrangement_cell_complex_shortcut_attempt = retained_arrangement_attempt
                .filter(|attempt| {
                    attempt.stage == ExactArrangementBooleanStage::Materialized
                        && attempt.decline.is_none()
                        && attempt.materialized_shortcut
                            == Some(ExactBooleanShortcutKind::ArrangementCellComplex)
                });
            let arrangement_cell_complex_shortcut_certified = arrangement_cell_complex_shortcuts
                .materializes_operation(request.operation)
                && retained_arrangement_cell_complex_shortcut_attempt.is_some();
            if arrangement_cell_complex_shortcut_certified {
                retained_arrangement_cell_complex_shortcut_attempt.cloned()
            } else if let Some(attempt) = retained_arrangement_attempt {
                Some(attempt.clone())
            } else if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_))
                || reject_boundary_evidence_request
                || request.validation == ExactMeshValidationPolicy::CLOSED
            {
                None
            } else if let Some(arrangement) = retained_regularized_arrangement {
                Some(
                    match run_arrangement_cell_complex_attempt_from_arrangement(
                        arrangement,
                        left,
                        right,
                        request,
                        ExactRegularizationPolicy::REGULARIZED_SOLID,
                        true,
                    )? {
                        ArrangementCellComplexOutcome::Materialized(_, attempt)
                        | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
                    },
                )
            } else {
                let arrangement =
                    ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
                        graph.clone(),
                        left,
                        right,
                        ExactRegularizationPolicy::REGULARIZED_SOLID,
                    )?;
                Some(
                    match run_arrangement_cell_complex_attempt_from_arrangement(
                        &arrangement,
                        left,
                        right,
                        request,
                        ExactRegularizationPolicy::REGULARIZED_SOLID,
                        true,
                    )? {
                        ArrangementCellComplexOutcome::Materialized(_, attempt)
                        | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
                    },
                )
            }
        }
    };
    let winding_evidence = match winding_evidence_report_for_request_from_graph_and_attempt(
        graph,
        left,
        right,
        request,
        arrangement_attempt.as_ref(),
        &arrangement_cell_complex_shortcuts,
    ) {
        Ok(report) => report,
        Err(_) => {
            let geometry = graph.face_split_geometry_plan(left, right)?;
            let region_plan = geometry.region_plan(left, right);
            let region_classifications =
                checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
            let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
            winding_evidence_report(
                request.operation,
                ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
                graph.has_unknowns(),
                graph.face_pairs.len(),
                graph.event_count(),
                region_plan.regions.len(),
                region_classifications,
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right)?,
            )
        }
    };
    Ok(ExactBooleanCertificationSet {
        trivial,
        regularized_solid,
        refinement,
        boundary_touching,
        open_surface_disjoint,
        adjacent_union_completion,
        identical,
        same_surface,
        closed_winding_left_in_right,
        closed_winding_right_in_left,
        convex_left_in_right,
        convex_right_in_left,
        convex_capabilities,
        arrangement_cell_complex_shortcuts,
        planar_arrangement,
        winding_evidence,
        volumetric_boundary_closure,
        arrangement_attempt,
    })
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
