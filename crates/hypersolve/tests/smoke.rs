use hyperreal::{Rational, Real, SymbolicDependencyMask};
use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_prepared};
use hypersolve::{
    BatchCandidateStatus, CandidateCertificationConfig, CandidateResidualBall,
    CertifiedCandidateStatus, Constraint, ConstraintKind, ConvergenceReason, DenseLinearBackend,
    DirectAffineSystemStatus, DomainCheckKind, DomainCheckStatus, ExactAffineRankStatus,
    ExactBranchStatus, Expr, ExprDegree, IntervalBoxCertificationPackage,
    IntervalBoxCertificationStatus, LinearAdapterKind, LinearAdapterPrecision, LinearBackend,
    MultivariateQuadraticKrawczykStatus, PreparedProblem, PreparedSolverBlock, Problem,
    ProposalEngineKind, ProposalEnginePrecision, RootIsolationStatus, RootMultiplicityStatus,
    SketchConstraintKind, SketchConstructionCertificateStatus, SketchDegeneracyKind,
    SketchDegeneracyStatus, SketchEntityHandle, SketchEntityKind, SketchGeneratedRowStatus,
    SketchParameterDomain, SketchParameterDomainKind, SketchParameterDomainStatus,
    SketchResidualFormKind, SketchResidualFormRole, SketchResidualFormsStatus,
    SketchResidualStrategy, SketchRoundTripMetadata, SketchRoundTripRole, SketchSolveProblem,
    SolverBlockRowKind, SolverConfig, SolverPoint2, SolverState, SymbolId, VariableBall,
    analyze_exact_affine_rank, apply_equality_substitution_classes, apply_equality_substitutions,
    build_equality_substitution_classes, certify_affine_interval_candidate,
    certify_affine_krawczyk_box, certify_candidate, certify_candidate_batch,
    certify_candidate_domains, certify_candidate_with_config,
    certify_candidate_with_residual_balls, certify_direct_univariate_quadratic_roots,
    certify_interval_box_candidate, certify_multivariate_quadratic_interval_candidate,
    certify_multivariate_quadratic_krawczyk_box, certify_quadratic_interval_candidate,
    certify_sketch_construction, certify_univariate_quadratic_alpha,
    certify_univariate_quadratic_krawczyk_box, context_from_problem,
    eliminate_affine_rows_with_substitution_classes,
    enumerate_direct_univariate_quadratic_branches, evaluate_residuals, facts_depend_on_symbol,
    find_equality_substitutions, isolate_univariate_polynomial_roots, point_coincidence_equations,
    preflight_sketch_degeneracies, preflight_sketch_parameter_domains,
    replay_sketch_compatibility_fixture, report_lossy_adapter_only_candidate,
    sketch_compatibility_fixtures, sketch_distance_builders, sketch_incidence_builders,
    sketch_objective_builders, sketch_orientation_builders, sketch_range_builders,
    solve_damped_least_squares, solve_direct_affine_equalities, solve_direct_affine_system,
    solve_direct_univariate_quadratic_equalities, squared_distance_equation,
    tangent_parallel_equation, tangent_same_direction_constraint, validate_equality_substitutions,
};

fn real(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn edge_real(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

#[test]
fn sketch_problem_lowers_semantic_constraints_to_exact_replay_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(3), real(4));
    let c = sketch.add_point2d("c", real(3), real(0));
    let distance = sketch.add_distance("five", real(5));
    let line = sketch.add_line_segment2("base", a, c);
    let circle = sketch.add_circle2("circle", a, distance);

    let coincidence = sketch.add_points_coincident("c equals b", c, b);
    let length = sketch.add_point_point_distance("a b length", a, b, distance);
    let horizontal = sketch.add_horizontal("base horizontal", line);
    let incidence = sketch.add_point_on_circle("b on circle", b, circle);

    let report = sketch.lower_to_problem();

    assert!(report.all_generated());
    assert_eq!(report.problem.variables.len(), 7);
    assert_eq!(report.problem.constraints.len(), 5);
    assert_eq!(
        report
            .rows
            .iter()
            .map(|row| row.constraint)
            .collect::<Vec<_>>(),
        vec![coincidence, coincidence, length, horizontal, incidence]
    );
    assert_eq!(
        report.rows[2].strategy,
        Some(SketchResidualStrategy::SquaredDistance)
    );
    assert_eq!(
        report.rows[4].strategy,
        Some(SketchResidualStrategy::SquaredIncidence)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation {
            sign: hyperreal::RealSign::Negative,
            ..
        }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
}

#[test]
fn sketch_lowering_reports_bad_handles_and_reference_rows() {
    let mut sketch = SketchSolveProblem::new();
    let point = sketch.add_point2d("p", real(0), real(0));
    let distance = sketch.add_distance("d", real(1));
    sketch.add_constraint(
        "reference distance",
        SketchConstraintKind::PointPointDistance {
            a: point,
            b: point,
            distance,
        },
        true,
        true,
    );
    sketch.add_points_coincident("missing point", point, SketchEntityHandle(999));
    sketch.add_horizontal("wrong entity", distance);

    let report = sketch.lower_to_problem();

    assert_eq!(report.problem.constraints.len(), 0);
    assert_eq!(report.rows.len(), 3);
    assert_eq!(
        report.rows[0].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        report.rows[1].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert!(matches!(
        report.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind { .. }
    ));
}

#[test]
fn sketch_retains_extended_solvespace_entity_carriers_without_forcing_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point2d("p0", real(0), real(0));
    let p1 = sketch.add_point2d("p1", real(1), real(0));
    let p2 = sketch.add_point2d("p2", real(2), real(1));
    let p3 = sketch.add_point2d("p3", real(3), real(1));
    let radius = sketch.add_distance("radius", real(1));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let arc = sketch.add_arc_of_circle2("arc", p0, p1, p2, radius);
    let normal2 = sketch.add_normal2d("direction", real(1), real(0));

    assert!(matches!(
        &sketch.entities()[workplane.0 as usize].kind,
        SketchEntityKind::Workplane(_)
    ));
    assert!(matches!(
        &sketch.entities()[cubic.0 as usize].kind,
        SketchEntityKind::Cubic2(_)
    ));
    assert!(matches!(
        &sketch.entities()[arc.0 as usize].kind,
        SketchEntityKind::ArcOfCircle2(_)
    ));
    assert!(matches!(
        &sketch.entities()[normal2.0 as usize].kind,
        SketchEntityKind::Normal2D(_)
    ));

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.variables.len(), sketch.parameters().len());
    assert_eq!(lowered.problem.constraints.len(), 0);
    assert!(lowered.rows.is_empty());
}

#[test]
fn sketch_family_builders_report_strategy_and_lower_to_matching_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(5), real(0));
    let distance = sketch.add_distance("five", real(5));
    let line = sketch.add_line_segment2("line", a, b);
    let circle = sketch.add_circle2("circle", a, distance);

    let coincident = sketch_incidence_builders::points_coincident(&mut sketch, "same", a, a);
    let length =
        sketch_distance_builders::point_point_distance(&mut sketch, "length", a, b, distance);
    let horizontal = sketch_orientation_builders::horizontal(&mut sketch, "horizontal", line);
    let incidence = sketch_incidence_builders::point_on_circle(&mut sketch, "on circle", b, circle);

    assert_eq!(
        coincident.family,
        hypersolve::SketchConstraintFamily::Incidence
    );
    assert_eq!(length.family, hypersolve::SketchConstraintFamily::Distance);
    assert_eq!(
        horizontal.family,
        hypersolve::SketchConstraintFamily::Orientation
    );
    assert_eq!(incidence.strategy, SketchResidualStrategy::SquaredIncidence);

    let lowered = sketch.lower_to_problem();

    assert!(lowered.all_generated());
    assert_eq!(
        lowered
            .rows
            .iter()
            .map(|row| row.constraint)
            .collect::<Vec<_>>(),
        vec![
            coincident.handle,
            coincident.handle,
            length.handle,
            horizontal.handle,
            incidence.handle,
        ]
    );
    assert_eq!(
        lowered.rows[2].strategy,
        Some(SketchResidualStrategy::SquaredDistance)
    );
    assert_eq!(
        lowered.rows[3].strategy,
        Some(SketchResidualStrategy::CoordinateEquality)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(certification.all_satisfied());
}

#[test]
fn sketch_distance_constraints_retain_exact_and_proposal_residual_forms() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(3), real(4));
    let distance = sketch.add_distance("five", real(5));
    let length =
        sketch_distance_builders::point_point_distance(&mut sketch, "length", a, b, distance);

    let forms = sketch.residual_forms_for_constraint(length.handle);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::SquaredDistancePolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::SquaredDistance)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::TrueDistanceProposal
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ProposalOnly);
    assert_eq!(forms.forms[1].strategy, None);

    let context = context_from_problem(&sketch.lower_to_problem().problem);
    assert_eq!(
        forms.forms[0]
            .residual
            .eval_real(context.bindings())
            .unwrap(),
        Real::zero()
    );
    assert_eq!(
        forms.forms[1]
            .residual
            .eval_real(context.bindings())
            .unwrap(),
        Real::zero()
    );
}

#[test]
fn sketch_residual_form_reports_reject_unsupported_and_bad_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let point = sketch.add_point2d("point", real(0), real(0));
    let distance = sketch.add_distance("distance", real(1));
    let horizontal = sketch.add_horizontal("not multi-form", distance);
    let bad = sketch_distance_builders::point_point_distance(
        &mut sketch,
        "bad",
        point,
        distance,
        distance,
    );

    assert_eq!(
        sketch.residual_forms_for_constraint(horizontal).status,
        SketchResidualFormsStatus::UnsupportedConstraint
    );

    let bad_forms = sketch.residual_forms_for_constraint(bad.handle);
    assert_eq!(bad_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(bad_forms.forms.is_empty());
    assert!(!bad_forms.diagnostics.is_empty());
    assert_eq!(
        sketch
            .residual_forms_for_constraint(hypersolve::SketchConstraintHandle(999))
            .status,
        SketchResidualFormsStatus::MissingConstraint(hypersolve::SketchConstraintHandle(999))
    );
}

#[test]
fn sketch_compatibility_fixtures_are_license_clean_and_exactly_certified() {
    let fixtures = sketch_compatibility_fixtures();

    assert_eq!(fixtures.len(), 3);
    assert!(fixtures.iter().all(|fixture| {
        fixture.source.starts_with("Hyper-authored")
            && !fixture.source.to_ascii_lowercase().contains("copied")
    }));

    for fixture in &fixtures {
        let replay = replay_sketch_compatibility_fixture(fixture);
        assert_eq!(replay.name, fixture.name);
        assert_eq!(replay.kind, fixture.kind);
        assert!(replay.is_certified_fixture(fixture.expected_generated_rows));
    }
}

#[test]
fn sketch_round_trip_metadata_preserves_editor_fields_without_forcing_proof_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(5), real(0));
    let distance = sketch.add_distance("displayed distance", real(5));
    let constraint = sketch.add_point_point_distance("reference length", a, b, distance);

    assert!(sketch.set_entity_metadata(
        a,
        SketchRoundTripMetadata {
            display_label: Some("construction origin".to_owned()),
            comment: Some("imported construction point".to_owned()),
            role: SketchRoundTripRole::Construction,
            ..SketchRoundTripMetadata::default()
        },
    ));
    assert!(sketch.set_parameter_metadata(
        sketch.parameters()[0].handle,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            display_label: Some("a.x".to_owned()),
            ..SketchRoundTripMetadata::default()
        },
    ));
    assert!(sketch.set_constraint_metadata(
        constraint,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            display_label: Some("5.000 mm".to_owned()),
            comment: Some("display/reference dimension".to_owned()),
            role: SketchRoundTripRole::ReferenceDimension,
            lossy_adapter_label: Some("SolveSpace-compatible UI dimension".to_owned()),
        },
    ));
    assert!(!sketch.set_constraint_metadata(
        hypersolve::SketchConstraintHandle(999),
        SketchRoundTripMetadata::default(),
    ));

    assert_eq!(
        sketch.entities()[a.0 as usize].metadata.role,
        SketchRoundTripRole::Construction
    );
    assert_eq!(
        sketch.parameters()[0].metadata.source_unit.as_deref(),
        Some("mm")
    );
    assert!(sketch.constraints()[constraint.0 as usize].reference);
    assert_eq!(
        sketch.constraints()[constraint.0 as usize]
            .metadata
            .lossy_adapter_label
            .as_deref(),
        Some("SolveSpace-compatible UI dimension")
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 1);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
}

#[test]
fn sketch_range_and_soft_objective_builders_lower_to_exact_row_kinds() {
    let mut sketch = SketchSolveProblem::new();
    let parameter = sketch.add_parameter("t", real(3));
    let range = sketch_range_builders::parameter_range(
        &mut sketch,
        "bounded t",
        parameter,
        Some(real(1)),
        Some(real(5)),
    );
    let objective = sketch_objective_builders::stay_near_parameter(
        &mut sketch,
        "stay near",
        parameter,
        real(3),
        real(7),
    );

    assert_eq!(range.family, hypersolve::SketchConstraintFamily::Range);
    assert_eq!(
        objective.family,
        hypersolve::SketchConstraintFamily::Objective
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.problem.constraints[0].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        lowered.problem.constraints[1].kind,
        ConstraintKind::LessOrEqual
    );
    assert_eq!(lowered.problem.constraints[2].kind, ConstraintKind::Soft);
    assert_eq!(lowered.problem.constraints[2].weight, real(7));
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::ParameterRange)
    );
    assert_eq!(
        lowered.rows[2].strategy,
        Some(SketchResidualStrategy::SoftObjective)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(certification.all_satisfied());
}

#[test]
fn sketch_parameter_ranges_report_empty_ranges_and_stale_parameters_explicitly() {
    let mut sketch = SketchSolveProblem::new();
    let parameter = sketch.add_parameter("t", real(0));
    sketch.add_parameter_range("empty", parameter, None, None);
    sketch.add_parameter_range(
        "stale",
        hypersolve::SketchParameterHandle(999),
        Some(real(0)),
        None,
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::MissingParameter(hypersolve::SketchParameterHandle(999))
    );
}

#[test]
fn sketch_parameter_domain_preflight_certifies_valid_invalid_and_empty_bounds() {
    let mut sketch = SketchSolveProblem::new();
    let positive = sketch.add_parameter("positive radius", real(3));
    let negative = sketch.add_parameter("negative distance", real(-1));
    let zero = sketch.add_parameter("zero length", real(0));
    let angle = sketch.add_parameter("angle", real(5));
    let periodic = sketch.add_parameter("periodic", real(17));

    assert!(sketch.add_parameter_domain(positive, SketchParameterDomain::Positive));
    assert!(sketch.add_parameter_domain(negative, SketchParameterDomain::Nonnegative));
    assert!(sketch.add_parameter_domain(zero, SketchParameterDomain::Nonzero));
    assert!(sketch.add_parameter_domain(
        angle,
        SketchParameterDomain::AngularRange {
            lower: real(0),
            upper: real(4),
        },
    ));
    assert!(sketch.add_parameter_domain(
        periodic,
        SketchParameterDomain::Periodic { period: real(0) },
    ));
    assert!(sketch.add_parameter_domain(
        positive,
        SketchParameterDomain::Bounded {
            lower: Some(real(5)),
            upper: Some(real(4)),
        },
    ));

    let report = preflight_sketch_parameter_domains(&sketch);

    assert_eq!(report.parameter_count, 5);
    assert_eq!(report.checks.len(), 6);
    assert_eq!(report.certified_valid_checks, 1);
    assert_eq!(report.certified_invalid_checks, 5);
    assert_eq!(report.unknown_checks, 0);
    assert!(report.has_certified_invalid_domain());
    assert!(!report.all_certified_valid());
    assert_eq!(report.checks[0].kind, SketchParameterDomainKind::Positive);
    assert_eq!(
        report.checks[0].status,
        SketchParameterDomainStatus::CertifiedValid
    );
    assert_eq!(
        report.checks[1].status,
        SketchParameterDomainStatus::CertifiedInvalid
    );
}

#[test]
fn sketch_parameter_domain_lock_marks_lowered_variable_fixed() {
    let mut sketch = SketchSolveProblem::new();
    let parameter = sketch.add_parameter("locked", real(11));

    assert!(sketch.lock_parameter(parameter));
    assert!(!sketch.lock_parameter(hypersolve::SketchParameterHandle(999)));
    let report = preflight_sketch_parameter_domains(&sketch);
    let lowered = sketch.lower_to_problem();

    assert_eq!(report.checks.len(), 1);
    assert_eq!(report.checks[0].kind, SketchParameterDomainKind::Locked);
    assert_eq!(
        report.checks[0].status,
        SketchParameterDomainStatus::CertifiedValid
    );
    assert!(lowered.problem.variables[0].fixed);
}

#[test]
fn sketch_degeneracy_preflight_explains_entity_level_degeneracies() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let same = sketch.add_point2d("same", real(0), real(0));
    let unit_x = sketch.add_point2d("unit x", real(1), real(0));
    let zero_distance = sketch.add_distance("zero radius", real(0));
    let unit_normal = sketch.add_normal2d("unit normal", real(1), real(0));
    let nonunit_normal = sketch.add_normal3d("bad normal", real(2), real(0), real(0), real(0));
    let zero_line = sketch.add_line_segment2("zero line", origin, same);
    let good_line = sketch.add_line_segment2("good line", origin, unit_x);
    let zero_circle = sketch.add_circle2("zero circle", origin, zero_distance);
    let full_arc = sketch.add_arc_of_circle2("full arc", origin, same, same, zero_distance);
    let singular_workplane = sketch.add_workplane("singular wp", origin, nonunit_normal);
    let good_workplane = sketch.add_workplane("good wp", origin, unit_normal);

    let report = preflight_sketch_degeneracies(&sketch);

    assert_eq!(report.entity_count, sketch.entities().len());
    assert_eq!(report.certified_degenerate_checks, 5);
    assert_eq!(report.invalid_reference_checks, 0);
    assert!(report.has_certified_degeneracy());
    assert!(report.checks.iter().any(|check| {
        check.entity == zero_line
            && check.kind == SketchDegeneracyKind::ZeroLengthLineSegment2
            && check.status == SketchDegeneracyStatus::CertifiedDegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == good_line
            && check.kind == SketchDegeneracyKind::ZeroLengthLineSegment2
            && check.status == SketchDegeneracyStatus::CertifiedNondegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == zero_circle
            && check.kind == SketchDegeneracyKind::ZeroRadiusCircle2
            && check.status == SketchDegeneracyStatus::CertifiedDegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == full_arc
            && check.kind == SketchDegeneracyKind::FullCircleArc2
            && check.status == SketchDegeneracyStatus::CertifiedDegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == nonunit_normal
            && check.kind == SketchDegeneracyKind::NonunitNormal3
            && check.status == SketchDegeneracyStatus::CertifiedDegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == singular_workplane
            && check.kind == SketchDegeneracyKind::SingularWorkplane
            && check.status == SketchDegeneracyStatus::CertifiedDegenerate
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == good_workplane
            && check.kind == SketchDegeneracyKind::SingularWorkplane
            && check.status == SketchDegeneracyStatus::CertifiedNondegenerate
    }));
}

#[test]
fn sketch_degeneracy_preflight_reports_stale_and_wrong_references() {
    let mut sketch = SketchSolveProblem::new();
    let point = sketch.add_point2d("p", real(0), real(0));
    let distance = sketch.add_distance("d", real(1));
    let stale_line = sketch.add_line_segment2("stale line", point, SketchEntityHandle(999));
    let wrong_circle = sketch.add_circle2("wrong circle", point, point);
    let wrong_workplane = sketch.add_workplane("wrong wp", point, distance);

    let report = preflight_sketch_degeneracies(&sketch);

    assert_eq!(report.invalid_reference_checks, 3);
    assert!(report.checks.iter().any(|check| {
        check.entity == stale_line
            && matches!(
                check.status,
                SketchDegeneracyStatus::MissingEntity {
                    handle: SketchEntityHandle(999)
                }
            )
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == wrong_circle
            && matches!(
                check.status,
                SketchDegeneracyStatus::WrongEntityKind {
                    handle,
                    expected: "distance"
                } if handle == point
            )
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == wrong_workplane
            && matches!(
                check.status,
                SketchDegeneracyStatus::WrongEntityKind {
                    handle,
                    expected: "normal"
                } if handle == distance
            )
    }));
}

#[test]
fn sketch_construction_certificate_bundles_replay_preflight_and_provenance() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(3), real(4));
    let distance = sketch.add_distance("distance", real(5));
    let constraint =
        sketch_distance_builders::point_point_distance(&mut sketch, "five", a, b, distance);
    assert!(sketch.set_constraint_metadata(
        constraint.handle,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            display_label: Some("five".to_owned()),
            role: SketchRoundTripRole::Construction,
            lossy_adapter_label: Some("importer".to_owned()),
            ..SketchRoundTripMetadata::default()
        },
    ));

    let certificate = certify_sketch_construction(&sketch);

    assert!(certificate.is_certified());
    assert_eq!(
        certificate.status,
        SketchConstructionCertificateStatus::Certified
    );
    assert!(certificate.lowering.all_generated());
    assert!(certificate.parameter_domains.all_certified_valid());
    assert!(!certificate.degeneracies.has_certified_degeneracy());
    assert!(certificate.residual_replay.all_satisfied());
    assert_eq!(certificate.traces.lossy_adapter_metadata_rows, 1);
    assert_eq!(certificate.traces.predicate_replay_rows, 0);
    assert_eq!(certificate.traces.interval_uniqueness_reports, 0);
    assert_eq!(certificate.traces.algebraic_root_references, 0);
    assert!(certificate.provenance.iter().any(|row| {
        row.constraint == Some(constraint.handle)
            && row.metadata.display_label.as_deref() == Some("five")
            && row.metadata.source_unit.as_deref() == Some("mm")
    }));
}

#[test]
fn sketch_construction_certificate_reports_preflight_lowering_and_replay_failures() {
    let mut invalid = SketchSolveProblem::new();
    let p = invalid.add_point2d("p", real(0), real(0));
    invalid.add_line_segment2("zero line", p, p);
    let invalid_certificate = certify_sketch_construction(&invalid);
    assert_eq!(
        invalid_certificate.status,
        SketchConstructionCertificateStatus::InvalidPreflight
    );
    assert!(invalid_certificate.degeneracies.has_certified_degeneracy());

    let mut incomplete = SketchSolveProblem::new();
    let q = incomplete.add_point2d("q", real(0), real(0));
    let distance = incomplete.add_distance("d", real(1));
    sketch_distance_builders::point_point_distance(
        &mut incomplete,
        "missing point",
        q,
        SketchEntityHandle(999),
        distance,
    );
    let incomplete_certificate = certify_sketch_construction(&incomplete);
    assert_eq!(
        incomplete_certificate.status,
        SketchConstructionCertificateStatus::LoweringIncomplete
    );
    assert!(!incomplete_certificate.lowering.all_generated());

    let mut rejected = SketchSolveProblem::new();
    let a = rejected.add_point2d("a", real(0), real(0));
    let b = rejected.add_point2d("b", real(3), real(4));
    let distance = rejected.add_distance("wrong distance", real(6));
    sketch_distance_builders::point_point_distance(&mut rejected, "six", a, b, distance);
    let rejected_certificate = certify_sketch_construction(&rejected);
    assert_eq!(
        rejected_certificate.status,
        SketchConstructionCertificateStatus::ReplayRejected
    );
    assert!(
        rejected_certificate
            .residual_replay
            .has_certified_violation()
    );
}

#[test]
fn expression_evaluates_with_symbol_binding() {
    let x = Expr::symbol(SymbolId(0), "x");
    let expr = (x.clone() * x) - Expr::int(9);
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality("x squared minus nine", expr));

    let context = context_from_problem(&problem);
    let residuals = evaluate_residuals(&problem, &context).unwrap();

    assert_eq!(residuals.len(), 1);
    assert_eq!(residuals[0].value, Real::zero());
}

#[test]
fn symbolic_derivative_solves_one_variable_linear_equation() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));

    let report = solve_damped_least_squares(SolverState {
        problem,
        config: SolverConfig {
            max_iterations: 8,
            residual_tolerance: edge_real(1.0e-6),
            step_tolerance: edge_real(1.0e-12),
            damping: edge_real(1.0e-9),
            proposal_engine: ProposalEngineKind::DampedLeastSquares,
        },
    });

    assert!(matches!(
        report.reason,
        ConvergenceReason::Converged | ConvergenceReason::StepTooSmall
    ));
    assert!(
        !report.linear_reports.is_empty(),
        "top-level solve reports should expose dense adapter diagnostics"
    );
    assert!(report.linear_reports.iter().all(|row| row.is_lossy()));
    assert_eq!(
        report.proposal_engine.requested,
        ProposalEngineKind::DampedLeastSquares
    );
    assert_eq!(
        report.proposal_engine.used,
        Some(ProposalEngineKind::DampedLeastSquares)
    );
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(
        report
            .linear_reports
            .iter()
            .all(|row| row.precision() == LinearAdapterPrecision::LossyF64)
    );
}

#[test]
fn dogleg_proposal_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::Dogleg,
        damping: real(1),
        max_iterations: 4,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_ne!(report.reason, ConvergenceReason::UnsupportedProposalEngine);
    assert_eq!(report.proposal_engine.requested, ProposalEngineKind::Dogleg);
    assert_eq!(
        report.proposal_engine.used,
        Some(ProposalEngineKind::Dogleg)
    );
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(!report.linear_reports.is_empty());
}

#[test]
fn powell_hybrid_proposal_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::PowellHybrid,
        damping: real(1),
        max_iterations: 4,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_ne!(report.reason, ConvergenceReason::UnsupportedProposalEngine);
    assert_eq!(
        report.proposal_engine.requested,
        ProposalEngineKind::PowellHybrid
    );
    assert_eq!(
        report.proposal_engine.used,
        Some(ProposalEngineKind::PowellHybrid)
    );
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(!report.linear_reports.is_empty());
}

#[test]
fn bfgs_proposal_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::Bfgs,
        damping: real(1),
        max_iterations: 4,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_ne!(report.reason, ConvergenceReason::UnsupportedProposalEngine);
    assert_eq!(report.proposal_engine.requested, ProposalEngineKind::Bfgs);
    assert_eq!(report.proposal_engine.used, Some(ProposalEngineKind::Bfgs));
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(!report.linear_reports.is_empty());
    assert!(report.linear_reports.iter().all(|row| row.is_lossy()));
}

#[test]
fn sqp_proposal_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::Sqp,
        damping: real(1),
        max_iterations: 4,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_ne!(report.reason, ConvergenceReason::UnsupportedProposalEngine);
    assert_eq!(report.proposal_engine.requested, ProposalEngineKind::Sqp);
    assert_eq!(report.proposal_engine.used, Some(ProposalEngineKind::Sqp));
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(!report.linear_reports.is_empty());
}

#[test]
fn levenberg_marquardt_proposal_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        max_iterations: 8,
        residual_tolerance: edge_real(1.0e-6),
        step_tolerance: edge_real(1.0e-12),
        damping: edge_real(1.0e-9),
        proposal_engine: ProposalEngineKind::LevenbergMarquardt,
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert!(matches!(
        report.reason,
        ConvergenceReason::Converged | ConvergenceReason::StepTooSmall
    ));
    assert_eq!(
        report.proposal_engine.requested,
        ProposalEngineKind::LevenbergMarquardt
    );
    assert_eq!(
        report.proposal_engine.used,
        Some(ProposalEngineKind::LevenbergMarquardt)
    );
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(
        !report.linear_reports.is_empty(),
        "Levenberg-Marquardt still crosses the dense f64 adapter boundary"
    );
    assert!(report.linear_reports.iter().all(|row| row.is_lossy()));
}

#[test]
fn modified_newton_least_squares_route_reports_named_lossy_adapter() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 4,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_ne!(report.reason, ConvergenceReason::UnsupportedProposalEngine);
    assert_eq!(
        report.proposal_engine.requested,
        ProposalEngineKind::ModifiedNewtonLeastSquares
    );
    assert_eq!(
        report.proposal_engine.used,
        Some(ProposalEngineKind::ModifiedNewtonLeastSquares)
    );
    assert_eq!(
        report.proposal_engine.precision,
        ProposalEnginePrecision::LossyF64
    );
    assert!(!report.linear_reports.is_empty());
}

#[test]
fn lossy_adapter_only_report_preserves_proposal_boundary_without_exact_replay() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_constraint(Constraint::equality(
        "x squared",
        x.clone().powi(2) - Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality("inactive", x - Expr::int(2)));
    problem.constraints[1].active = false;
    let prepared = PreparedProblem::new(&problem);

    let report = report_lossy_adapter_only_candidate(
        &prepared,
        hypersolve::ProposalEngineReport {
            requested: ProposalEngineKind::DampedLeastSquares,
            used: Some(ProposalEngineKind::DampedLeastSquares),
            precision: ProposalEnginePrecision::LossyF64,
            supported: true,
        },
    );

    assert_eq!(report.rows.len(), 1);
    assert_eq!(report.certified_satisfied_rows, 0);
    assert_eq!(report.certified_violation_rows, 0);
    assert_eq!(report.bounded_unknown_rows, 1);
    assert_eq!(report.lossy_adapter_only_rows, 1);
    assert!(report.has_only_uncertainty());
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::LossyAdapterOnly {
            requested: ProposalEngineKind::DampedLeastSquares,
            used: Some(ProposalEngineKind::DampedLeastSquares),
            precision: ProposalEnginePrecision::LossyF64,
        }
    ));
}

#[test]
fn candidate_batch_certification_reports_deterministic_failed_row_probes() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "x equals two",
        x.clone() - Expr::int(2),
    ));
    problem.add_constraint(Constraint {
        name: "x at most three".to_owned(),
        kind: ConstraintKind::LessOrEqual,
        residual: x - Expr::int(3),
        weight: real(1),
        active: true,
    });
    let prepared = PreparedProblem::new(&problem);
    let mut certified = context_from_problem(&problem);
    certified.bind(SymbolId(0), real(2));
    let mut rejected = context_from_problem(&problem);
    rejected.bind(SymbolId(0), real(4));
    let missing = hypersolve::EvaluationContext::default();

    let report = certify_candidate_batch(&prepared, &[rejected, certified, missing]);

    assert_eq!(report.candidate_count, 3);
    assert_eq!(report.certified_candidates, 1);
    assert_eq!(report.rejected_candidates, 1);
    assert_eq!(report.domain_failure_candidates, 1);
    assert!(report.has_certified_candidate());
    assert_eq!(report.candidates[0].candidate_index, 0);
    assert_eq!(report.candidates[0].status, BatchCandidateStatus::Rejected);
    assert_eq!(report.candidates[0].first_failed_constraint, Some(0));
    assert_eq!(report.candidates[0].violated_constraints, vec![0, 1]);
    assert_eq!(report.candidates[1].candidate_index, 1);
    assert_eq!(report.candidates[1].status, BatchCandidateStatus::Certified);
    assert!(report.candidates[1].certification.all_satisfied());
    assert_eq!(report.candidates[2].candidate_index, 2);
    assert_eq!(
        report.candidates[2].status,
        BatchCandidateStatus::DomainFailure
    );
    assert_eq!(report.candidates[2].domain_failure_constraints, vec![0, 1]);
}

#[test]
fn expression_facts_classify_affine_polynomial_and_transcendental_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");

    let affine = (x.clone() * Expr::int(3)) + Expr::int(7);
    let affine_facts = affine.structural_facts();
    assert_eq!(affine_facts.degree, ExprDegree::Polynomial(1));
    assert!(affine_facts.is_affine());
    assert!(affine_facts.dependencies.contains(&SymbolId(0)));

    let quadratic = x.clone() * y.clone();
    let quadratic_facts = quadratic.structural_facts();
    assert_eq!(quadratic_facts.degree, ExprDegree::Polynomial(2));
    assert!(quadratic_facts.dependencies.contains(&SymbolId(0)));
    assert!(quadratic_facts.dependencies.contains(&SymbolId(1)));

    let non_polynomial = (x.clone() / (y + Expr::int(1))).sin();
    assert_eq!(
        non_polynomial.structural_facts().degree,
        ExprDegree::NonPolynomial
    );
}

#[test]
fn expression_facts_preserve_real_symbolic_dependency_families() {
    let x = Expr::symbol(SymbolId(0), "x");
    let expression = Expr::real(Real::pi()) + x;
    let facts = expression.structural_facts();

    assert_eq!(facts.degree, ExprDegree::Polynomial(1));
    assert!(facts.dependencies.contains(&SymbolId(0)));
    assert!(
        facts
            .real_symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
}

#[test]
fn simplify_folds_exact_unary_endpoints_without_hiding_invalid_domains() {
    assert_eq!(Expr::zero().sin().simplify(), Expr::zero());
    assert_eq!(Expr::zero().cos().simplify(), Expr::int(1));
    assert_eq!(Expr::int(1).sqrt().simplify(), Expr::int(1));
    assert_eq!(Expr::int(1).log10().simplify(), Expr::zero());
    assert_eq!(Expr::int(1).ln().simplify(), Expr::zero());
    assert_eq!(Expr::zero().asin().simplify(), Expr::zero());
    assert_eq!(Expr::int(1).acosh().simplify(), Expr::zero());
    assert_eq!(Expr::zero().atanh().simplify(), Expr::zero());

    let invalid_sqrt = Expr::int(-1).sqrt().simplify();
    assert!(matches!(invalid_sqrt, Expr::Sqrt(_)));
    let invalid_log = Expr::zero().log10().simplify();
    assert!(matches!(invalid_log, Expr::Log10(_)));
    assert!(matches!(Expr::zero().ln().simplify(), Expr::Ln(_)));
    assert!(matches!(Expr::int(2).asin().simplify(), Expr::Asin(_)));
    assert!(matches!(Expr::int(-2).acos().simplify(), Expr::Acos(_)));
    assert!(matches!(Expr::zero().acosh().simplify(), Expr::Acosh(_)));
    assert!(matches!(Expr::int(1).atanh().simplify(), Expr::Atanh(_)));
}

#[test]
fn prepared_problem_caches_residual_dependency_and_sparsity_facts() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "affine x row",
        x.clone() * Expr::int(2) + Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "quadratic coupling",
        x.clone() * y.clone(),
    ));
    problem.add_constraint(Constraint::equality("constant", Expr::int(7)));

    let prepared = PreparedProblem::new(&problem);
    let facts = prepared.facts();

    assert_eq!(facts.variable_count, 2);
    assert_eq!(facts.constraint_count, 3);
    assert_eq!(facts.active_constraint_count, 3);
    assert_eq!(facts.affine_active_rows, 1);
    assert_eq!(facts.prepared_affine_active_rows, 2);
    assert_eq!(facts.polynomial_active_rows, 3);
    assert_eq!(facts.prepared_quadratic_active_rows, 1);
    assert_eq!(facts.non_polynomial_active_rows, 0);
    assert_eq!(facts.known_zero_constant_active_rows, 0);
    assert_eq!(facts.known_nonzero_constant_active_rows, 1);
    assert_eq!(facts.unknown_sign_constant_active_rows, 0);
    assert!(facts.has_known_nonzero_constant_residual());
    assert_eq!(facts.structural_jacobian_nonzeros, 3);
    assert!(!facts.all_active_rows_prepared_affine());

    assert_eq!(prepared.constraints()[0].dependent_columns, vec![0]);
    assert_eq!(prepared.constraints()[1].dependent_columns, vec![0, 1]);
    assert_eq!(
        prepared.constraints()[2].dependent_columns,
        Vec::<usize>::new()
    );
    assert_eq!(
        prepared.constraints()[2].residual_constant_sign,
        Some(hyperreal::RealSign::Positive)
    );
    assert!(prepared.constraints()[2].is_known_nonzero_constant_row());
    assert_eq!(prepared.jacobian_sparsity()[0], vec![true, false]);
    assert_eq!(prepared.jacobian_sparsity()[1], vec![true, true]);
    assert_eq!(prepared.jacobian_sparsity()[2], vec![false, false]);
    assert!(prepared.affine_residuals()[0].is_some());
    assert!(prepared.affine_residuals()[1].is_none());
    assert!(prepared.affine_residuals()[2].is_some());
    assert!(prepared.quadratic_residuals()[1].is_some());
    assert!(facts_depend_on_symbol(
        &prepared.constraints()[1].residual,
        SymbolId(1)
    ));
}

#[test]
fn prepared_problem_extracts_multivariate_quadratic_residuals() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_variable("y", real(3));
    problem.add_constraint(Constraint::equality(
        "mixed quadratic",
        x.clone() * y.clone() * Expr::int(5) + x.clone().powi(2) * Expr::int(2)
            - y.clone() * Expr::int(7)
            + Expr::int(11),
    ));
    let prepared = PreparedProblem::new(&problem);
    let block = PreparedSolverBlock::new(&prepared);
    let quadratic = prepared.quadratic_residuals()[0]
        .as_ref()
        .expect("mixed quadratic row should be prepared");

    assert_eq!(prepared.facts().prepared_quadratic_active_rows, 1);
    assert_eq!(block.facts().prepared_quadratic_row_count, 1);
    assert_eq!(block.facts().prepared_univariate_quadratic_row_count, 0);
    assert_eq!(quadratic.constant(), &real(11));
    assert_eq!(quadratic.linear_terms().len(), 1);
    assert_eq!(quadratic.linear_terms()[0].symbol, SymbolId(1));
    assert_eq!(quadratic.linear_terms()[0].coefficient, real(-7));
    assert_eq!(quadratic.quadratic_terms().len(), 2);
    assert_eq!(
        quadratic
            .eval_real(
                problem.variables.as_slice(),
                context_from_problem(&problem).bindings()
            )
            .unwrap(),
        real(28)
    );
}

#[test]
fn candidate_replay_uses_prepared_quadratic_residual_blocks() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_variable("y", real(4));
    problem.add_constraint(Constraint::equality(
        "quadratic replay block",
        x.clone().powi(2) + x.clone() * y.clone() * Expr::int(2) + y.clone().powi(2)
            - Expr::int(49),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    assert!(prepared.quadratic_residuals()[0].is_some());
    assert_eq!(
        prepared
            .evaluate_constraint_residual(0, &context)
            .expect("prepared quadratic replay should succeed"),
        Real::zero()
    );
    assert!(
        certify_candidate(&prepared, &context).all_satisfied(),
        "candidate certification should use the retained quadratic replay route"
    );
}

#[test]
fn geometry_domain_builds_exact_distance_and_tangent_residuals() {
    let mut problem = Problem::default();
    let ax = problem.add_variable("ax", real(0));
    let ay = problem.add_variable("ay", real(0));
    let bx = problem.add_variable("bx", real(3));
    let by = problem.add_variable("by", real(4));
    let a = SolverPoint2::new(ax, ay);
    let b = SolverPoint2::new(bx, by);

    problem.add_constraint(squared_distance_equation("3-4-5 distance", a, b, real(25)));
    problem.add_constraint(tangent_parallel_equation("parallel tangent", a, b));
    problem.add_constraint(tangent_same_direction_constraint(
        "same direction tangent",
        b,
        b,
    ));

    let prepared = PreparedProblem::new(&problem);
    assert_eq!(prepared.facts().polynomial_active_rows, 3);
    assert_eq!(prepared.facts().non_polynomial_active_rows, 0);
    assert!(certify_candidate(&prepared, &context_from_problem(&problem)).all_satisfied());

    let mut reversed = Problem::default();
    let cx = reversed.add_variable("cx", real(3));
    let cy = reversed.add_variable("cy", real(4));
    let dx = reversed.add_variable("dx", real(-6));
    let dy = reversed.add_variable("dy", real(-8));
    reversed.add_constraint(tangent_same_direction_constraint(
        "opposite tangent rejected",
        SolverPoint2::new(cx, cy),
        SolverPoint2::new(dx, dy),
    ));
    assert!(
        certify_candidate(
            &PreparedProblem::new(&reversed),
            &context_from_problem(&reversed)
        )
        .has_certified_violation()
    );
}

#[test]
fn geometry_domain_keeps_point_coincidence_affine_rows_split() {
    let mut problem = Problem::default();
    let ax = problem.add_variable("ax", real(7));
    let ay = problem.add_variable("ay", real(-2));
    let bx = problem.add_variable("bx", real(7));
    let by = problem.add_variable("by", real(-2));

    for constraint in point_coincidence_equations(
        "join endpoint",
        SolverPoint2::new(ax, ay),
        SolverPoint2::new(bx, by),
    ) {
        problem.add_constraint(constraint);
    }

    let prepared = PreparedProblem::new(&problem);
    assert_eq!(prepared.facts().affine_active_rows, 2);
    assert_eq!(prepared.affine_residuals().len(), 2);
    assert!(prepared.affine_residuals().iter().all(Option::is_some));
    assert!(certify_candidate(&prepared, &context_from_problem(&problem)).all_satisfied());
}

#[test]
fn prepared_affine_residual_preserves_coefficients_and_exact_product_sum_eval() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_variable("y", real(7));
    problem.add_constraint(Constraint::equality(
        "affine block",
        x.clone() * Expr::int(2) - y.clone() * Expr::int(3) + Expr::int(11),
    ));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);
    let affine = prepared.affine_residuals()[0]
        .as_ref()
        .expect("affine row should be prepared");

    assert!(prepared.facts().all_active_rows_prepared_affine());
    assert_eq!(affine.coefficients(), &[real(2), real(-3)]);
    assert_eq!(affine.constant(), &real(11));
    assert_eq!(affine.nonzero_coefficient_count(), 2);
    assert!(affine.coefficient_exact().all_exact_rational);
    assert!(affine.is_exact_rational());

    let prepared_value = affine
        .eval_real(&problem.variables, context.bindings())
        .expect("prepared affine eval should succeed");
    let ordinary_value = problem.constraints[0]
        .residual
        .eval_real(context.bindings())
        .expect("ordinary expression eval should succeed");

    assert_eq!(prepared_value, ordinary_value);
    assert_eq!(
        prepared.evaluate_residuals(&context).unwrap()[0].value,
        real(0)
    );
}

#[test]
fn prepared_affine_residual_rejects_stale_model_shape() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_constraint(Constraint::equality("affine block", x + Expr::int(1)));
    let prepared = PreparedProblem::new(&problem);
    let affine = prepared.affine_residuals()[0]
        .as_ref()
        .expect("affine row should be prepared");

    let error = affine
        .eval_real(&[], context_from_problem(&problem).bindings())
        .expect_err("stale prepared affine row must not evaluate against a different model shape");

    match error {
        hypersolve::ExprEvalError::PreparedShapeMismatch {
            expected_coefficients,
            actual_variables,
        } => {
            assert_eq!(expected_coefficients, 1);
            assert_eq!(actual_variables, 0);
        }
        other => panic!("unexpected eval error: {other:?}"),
    }
}

#[test]
fn prepared_problem_classifies_constant_residual_signs_without_evaluation() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("known zero", Expr::zero()));
    problem.add_constraint(Constraint::equality("known negative", Expr::int(-5)));
    problem.add_constraint(Constraint::equality(
        "constant expression tree",
        Expr::int(2) - Expr::int(2),
    ));
    problem.add_constraint(Constraint::equality("variable row", x - Expr::int(1)));

    let prepared = PreparedProblem::new(&problem);
    let facts = prepared.facts();

    assert_eq!(facts.active_constraint_count, 4);
    assert_eq!(facts.known_zero_constant_active_rows, 1);
    assert_eq!(facts.known_nonzero_constant_active_rows, 1);
    assert_eq!(facts.unknown_sign_constant_active_rows, 1);
    assert!(facts.has_known_nonzero_constant_residual());

    assert!(prepared.constraints()[0].is_known_zero_constant_row());
    assert_eq!(
        prepared.constraints()[1].residual_constant_sign,
        Some(hyperreal::RealSign::Negative)
    );
    assert!(prepared.constraints()[1].is_known_nonzero_constant_row());
    assert!(prepared.constraints()[2].is_constant_row());
    assert_eq!(prepared.constraints()[2].residual_constant_sign, None);
    assert!(!prepared.constraints()[3].is_constant_row());
    assert_eq!(prepared.constraints()[3].residual_constant_sign, None);
}

#[test]
fn prepared_solver_block_partitions_direct_affine_and_nonlinear_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(1));
    problem.add_variable("y", real(2));
    problem.add_constraint(Constraint::equality("zero", Expr::zero()));
    problem.add_constraint(Constraint::equality("contradiction", Expr::int(1)));
    problem.add_constraint(Constraint::equality("affine", x.clone() + Expr::int(3)));
    problem.add_constraint(Constraint::equality("quadratic", x.clone() * y.clone()));
    problem.add_constraint(Constraint {
        name: "trig".to_string(),
        kind: ConstraintKind::Equality,
        residual: x.sin(),
        weight: Real::one(),
        active: true,
    });

    let prepared = PreparedProblem::new(&problem);
    let block = PreparedSolverBlock::new(&prepared);
    let facts = block.facts();

    assert_eq!(facts.row_count, 5);
    assert_eq!(facts.active_row_count, 5);
    assert_eq!(facts.constant_row_count, 2);
    assert_eq!(facts.constant_contradiction_count, 1);
    assert_eq!(facts.prepared_affine_row_count, 1);
    assert_eq!(facts.polynomial_nonlinear_row_count, 1);
    assert_eq!(facts.prepared_univariate_quadratic_row_count, 0);
    assert_eq!(facts.non_polynomial_row_count, 1);
    assert_eq!(facts.nonlinear_proposal_row_count, 2);
    assert!(facts.has_exact_constant_contradiction());
    assert!(!facts.all_active_rows_affine_or_constant());

    assert_eq!(
        block.rows()[0].kind,
        SolverBlockRowKind::ConstantCertifiedZero
    );
    assert_eq!(
        block.rows()[1].kind,
        SolverBlockRowKind::ConstantCertifiedContradiction
    );
    assert_eq!(block.rows()[2].kind, SolverBlockRowKind::PreparedAffine);
    assert_eq!(block.rows()[3].kind, SolverBlockRowKind::Polynomial);
    assert_eq!(block.rows()[4].kind, SolverBlockRowKind::NonPolynomial);
}

#[test]
fn prepared_problem_extracts_univariate_quadratic_residuals() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_variable("y", real(5));
    problem.add_constraint(Constraint::equality(
        "univariate quadratic",
        x.clone() * x.clone() * Expr::int(2) - x.clone() * Expr::int(7) + Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality("cross term", x.clone() * y));
    problem.add_constraint(Constraint::equality(
        "prepared from pow",
        x.clone().powi(2) - Expr::int(9),
    ));

    let prepared = PreparedProblem::new(&problem);
    let block = PreparedSolverBlock::new(&prepared);
    let first = prepared.univariate_quadratic_residuals()[0]
        .as_ref()
        .expect("univariate quadratic row should be prepared");
    let third = prepared.univariate_quadratic_residuals()[2]
        .as_ref()
        .expect("powi(2) row should be prepared");

    assert_eq!(prepared.facts().polynomial_active_rows, 3);
    assert_eq!(
        prepared.facts().prepared_univariate_quadratic_active_rows,
        2
    );
    assert_eq!(block.facts().prepared_univariate_quadratic_row_count, 2);
    assert_eq!(first.symbol(), SymbolId(0));
    assert_eq!(first.quadratic(), &real(2));
    assert_eq!(first.linear(), &real(-7));
    assert_eq!(first.constant(), &real(4));
    assert_eq!(
        first
            .eval_real(
                problem.variables.as_slice(),
                context_from_problem(&problem).bindings()
            )
            .unwrap(),
        real(1)
    );
    assert!(prepared.univariate_quadratic_residuals()[1].is_none());
    assert_eq!(
        third
            .eval_real(
                problem.variables.as_slice(),
                context_from_problem(&problem).bindings()
            )
            .unwrap(),
        Real::zero()
    );
}

#[test]
fn univariate_quadratic_alpha_certifies_exact_simple_root_and_nearby_basin() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut exact_root = Problem::default();
    exact_root.add_variable("x", real(2));
    exact_root.add_constraint(Constraint::equality(
        "simple root",
        x.clone().powi(2) - Expr::int(4),
    ));
    let exact_report = certify_univariate_quadratic_alpha(
        &PreparedProblem::new(&exact_root),
        &context_from_problem(&exact_root),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(exact_report.examined_rows, 1);
    assert_eq!(
        exact_report.rows[0].status,
        hypersolve::UnivariateQuadraticAlphaStatus::ExactSimpleRoot
    );
    assert!(exact_report.all_examined_rows_certified());

    let mut near = Problem::default();
    near.add_variable("x", real(5));
    near.add_constraint(Constraint::equality(
        "near root",
        x.clone().powi(2) - Expr::int(24),
    ));
    let near_report = certify_univariate_quadratic_alpha(
        &PreparedProblem::new(&near),
        &context_from_problem(&near),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(
        near_report.rows[0].status,
        hypersolve::UnivariateQuadraticAlphaStatus::CertifiedSmaleAlphaBound
    );
    assert_eq!(near_report.rows[0].alpha_numerator, Some(real(1)));
    assert_eq!(near_report.rows[0].alpha_denominator, Some(real(100)));
    assert!(near_report.all_examined_rows_certified());
}

#[test]
fn univariate_quadratic_alpha_reports_multiple_root_and_failed_bound() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut multiple = Problem::default();
    multiple.add_variable("x", real(0));
    multiple.add_constraint(Constraint::equality("multiple root", x.clone().powi(2)));
    let multiple_report = certify_univariate_quadratic_alpha(
        &PreparedProblem::new(&multiple),
        &context_from_problem(&multiple),
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        multiple_report.rows[0].status,
        hypersolve::UnivariateQuadraticAlphaStatus::ExactMultipleRoot
    );
    assert!(!multiple_report.all_examined_rows_certified());

    let mut far = Problem::default();
    far.add_variable("x", real(1));
    far.add_constraint(Constraint::equality(
        "far from roots",
        x.clone().powi(2) - Expr::int(100),
    ));
    let far_report = certify_univariate_quadratic_alpha(
        &PreparedProblem::new(&far),
        &context_from_problem(&far),
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        far_report.rows[0].status,
        hypersolve::UnivariateQuadraticAlphaStatus::BoundFailed
    );
}

#[test]
fn univariate_quadratic_krawczyk_certifies_unique_root_and_reports_failures() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_constraint(Constraint::equality(
        "quadratic root",
        x.clone().powi(2) - Expr::int(4),
    ));

    let report = certify_univariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&problem),
        &context_from_problem(&problem),
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.examined_rows, 1);
    assert_eq!(report.certified_rows, 1);
    assert_eq!(
        report.rows[0].status,
        hypersolve::QuadraticKrawczykStatus::CertifiedUniqueRoot
    );
    assert_eq!(report.rows[0].step, Some(Real::zero()));
    assert_eq!(
        report.rows[0].remainder_radius,
        Some(Real::new(Rational::fraction(1, 2).unwrap()))
    );
    assert!(report.all_examined_rows_certified());

    let mut singular = Problem::default();
    singular.add_variable("x", real(0));
    singular.add_constraint(Constraint::equality("singular center", x.clone().powi(2)));
    let singular_report = certify_univariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&singular),
        &context_from_problem(&singular),
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        singular_report.rows[0].status,
        hypersolve::QuadraticKrawczykStatus::SingularOrUnsupportedDerivative
    );

    let mut far = Problem::default();
    far.add_variable("x", real(5));
    far.add_constraint(Constraint::equality(
        "far center",
        x.clone().powi(2) - Expr::int(4),
    ));
    let far_report = certify_univariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&far),
        &context_from_problem(&far),
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        far_report.rows[0].status,
        hypersolve::QuadraticKrawczykStatus::ImageOutsideBox
    );
}

#[test]
fn multivariate_quadratic_krawczyk_certifies_coupled_square_system() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(1));
    problem.add_variable("y", real(2));
    problem.add_constraint(Constraint::equality(
        "x squared plus y minus three",
        x.clone().powi(2) + y.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "x plus y squared minus five",
        x.clone() + y.clone().powi(2) - Expr::int(5),
    ));

    let report = certify_multivariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&problem),
        &context_from_problem(&problem),
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: Real::new(Rational::fraction(1, 10).unwrap()),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: Real::new(Rational::fraction(1, 10).unwrap()),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(
        report.status,
        MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot
    );
    assert!(report.certified_unique_root());
    assert_eq!(report.variable_count, 2);
    assert_eq!(report.equality_rows, 2);
    assert_eq!(report.residuals.len(), 2);
    assert_eq!(report.variables.len(), 2);
    assert!(report.variables.iter().all(|variable| {
        variable.step == Real::zero()
            && variable.image_radius <= variable.radius
            && variable.contraction_bound < Real::from(1)
    }));

    let tiny_box = certify_multivariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&problem),
        &context_from_problem(&problem),
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: real(0),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: real(0),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        tiny_box.status,
        MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot
    );

    let mut singular = Problem::default();
    singular.add_variable("x", real(0));
    singular.add_variable("y", real(0));
    singular.add_constraint(Constraint::equality("x squared", x.clone().powi(2)));
    singular.add_constraint(Constraint::equality("y squared", y.clone().powi(2)));
    let singular_report = certify_multivariate_quadratic_krawczyk_box(
        &PreparedProblem::new(&singular),
        &context_from_problem(&singular),
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: real(1),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: real(1),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        singular_report.status,
        MultivariateQuadraticKrawczykStatus::SingularOrUnsupportedPivot { pivot: 0 }
    );
}

#[test]
fn direct_quadratic_solver_returns_exact_root_candidates() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "two roots",
        x.clone().powi(2) - Expr::int(5) * x.clone() + Expr::int(6),
    ));
    problem.add_constraint(Constraint::equality(
        "double root",
        x.clone().powi(2) - Expr::int(4) * x.clone() + Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality(
        "no real roots",
        x.clone().powi(2) + Expr::int(1),
    ));

    let prepared = PreparedProblem::new(&problem);
    let solutions = solve_direct_univariate_quadratic_equalities(&prepared).unwrap();

    assert_eq!(solutions.len(), 3);
    assert_eq!(solutions[0].symbol, SymbolId(0));
    assert_eq!(solutions[0].roots, vec![real(3), real(2)]);
    assert_eq!(solutions[1].roots, vec![real(2)]);
    assert!(solutions[2].roots.is_empty());
}

#[test]
fn direct_quadratic_root_candidates_replay_full_problem_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "two roots",
        x.clone().powi(2) - Expr::int(5) * x.clone() + Expr::int(6),
    ));
    problem.add_constraint(Constraint::equality("select root two", x - Expr::int(2)));
    let prepared = PreparedProblem::new(&problem);
    let base = context_from_problem(&problem);

    let reports = certify_direct_univariate_quadratic_roots(&prepared, &base).unwrap();

    assert_eq!(reports.len(), 2);
    assert_eq!(reports[0].constraint_index, 0);
    assert_eq!(reports[0].symbol, SymbolId(0));
    assert_eq!(reports[0].root, Some(real(3)));
    assert_eq!(reports[1].root, Some(real(2)));
    let rejected = reports[0]
        .certification
        .as_ref()
        .expect("root candidate should replay");
    assert!(rejected.has_certified_violation());
    assert_eq!(
        reports[0].status,
        hypersolve::DirectQuadraticCandidateStatus::ReplayRejected
    );
    assert_eq!(
        reports[1].status,
        hypersolve::DirectQuadraticCandidateStatus::ReplayCertified
    );
    assert!(reports[1].certification.as_ref().unwrap().all_satisfied());
}

#[test]
fn exact_quadratic_branch_enumeration_reports_rejected_empty_and_unsupported_branches() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "two branches",
        x.clone().powi(2) - Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "select positive branch",
        x.clone() - Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "empty branch",
        y.clone().powi(2) + Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "unsupported affine row",
        y - Expr::int(2),
    ));
    let prepared = PreparedProblem::new(&problem);
    let report =
        enumerate_direct_univariate_quadratic_branches(&prepared, &context_from_problem(&problem));

    assert_eq!(report.certified_branches, 0);
    assert_eq!(report.rejected_branches, 2);
    assert_eq!(report.no_real_root_rows, 1);
    assert_eq!(report.unsupported_rows, 2);
    assert!(!report.has_certified_branch());
    assert_eq!(report.branches[0].status, ExactBranchStatus::ReplayRejected);
    assert_eq!(report.branches[0].root, Some(real(1)));
    assert!(
        report.branches[0]
            .certification
            .as_ref()
            .unwrap()
            .has_certified_violation()
    );
    assert_eq!(report.branches[1].status, ExactBranchStatus::ReplayRejected);
    assert_eq!(report.branches[1].root, Some(real(-1)));
    assert_eq!(report.branches[2].status, ExactBranchStatus::UnsupportedRow);
    assert_eq!(report.branches[3].status, ExactBranchStatus::NoRealRoots);
    assert_eq!(report.branches[4].status, ExactBranchStatus::UnsupportedRow);
}

#[test]
fn exact_quadratic_branch_enumeration_keeps_double_roots_as_one_branch() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(7));
    problem.add_constraint(Constraint::equality(
        "double branch",
        x.clone().powi(2) - Expr::int(4) * x + Expr::int(4),
    ));
    let prepared = PreparedProblem::new(&problem);
    let report =
        enumerate_direct_univariate_quadratic_branches(&prepared, &context_from_problem(&problem));

    assert_eq!(report.branches.len(), 1);
    assert_eq!(report.certified_branches, 1);
    assert_eq!(report.rejected_branches, 0);
    assert_eq!(report.branches[0].root, Some(real(2)));
    assert_eq!(report.branches[0].root_index, Some(0));
    assert_eq!(
        report.branches[0].status,
        ExactBranchStatus::ReplayCertified
    );
}

#[test]
fn root_isolation_sturm_reports_distinct_repeated_and_unsupported_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "three integer roots",
        x.clone().powi(3) - Expr::int(6) * x.clone().powi(2) + Expr::int(11) * x.clone()
            - Expr::int(6),
    ));
    problem.add_constraint(Constraint::equality(
        "repeated root package",
        (x.clone() - Expr::int(2)).powi(2) * (x.clone() + Expr::int(1)),
    ));
    problem.add_constraint(Constraint::equality(
        "no real roots",
        x.clone().powi(2) + Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality("multivariate unsupported", x * y));

    let reports = isolate_univariate_polynomial_roots(
        &PreparedProblem::new(&problem),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(reports.len(), 4);
    assert_eq!(reports[0].status, RootIsolationStatus::Isolated);
    assert_eq!(reports[0].symbol, Some(SymbolId(0)));
    assert_eq!(reports[0].degree, Some(3));
    assert_eq!(
        reports[0].multiplicity,
        Some(RootMultiplicityStatus::SquareFree)
    );
    assert_eq!(reports[0].intervals.len(), 3);
    assert!(
        reports[0]
            .intervals
            .iter()
            .all(|interval| interval.distinct_root_count == 1)
    );

    assert_eq!(reports[1].status, RootIsolationStatus::MultipleRoot);
    assert_eq!(
        reports[1].multiplicity,
        Some(RootMultiplicityStatus::RepeatedRootsDetected { gcd_degree: 1 })
    );
    assert_eq!(reports[1].intervals.len(), 2);

    assert_eq!(reports[2].status, RootIsolationStatus::NoRealRoots);
    assert!(reports[2].intervals.is_empty());

    assert_eq!(
        reports[3].status,
        RootIsolationStatus::UnsupportedCoefficient
    );
    assert!(reports[3].message.is_some());
}

#[test]
fn candidate_certification_replays_affine_rows_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_constraint(Constraint::equality(
        "x minus two",
        x.clone() - Expr::int(2),
    ));
    problem.add_constraint(Constraint::equality("x minus three", x - Expr::int(3)));

    let prepared = PreparedProblem::new(&problem);
    let report = certify_candidate(&prepared, &context_from_problem(&problem));

    assert_eq!(report.rows.len(), 2);
    assert_eq!(report.certified_satisfied_rows, 1);
    assert_eq!(report.certified_violation_rows, 1);
    assert!(!report.all_satisfied());
    assert!(report.has_certified_violation());
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        report.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn candidate_certification_handles_inequality_active_sets_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_constraint(Constraint {
        name: "x <= 3".to_string(),
        kind: ConstraintKind::LessOrEqual,
        residual: x.clone() - Expr::int(3),
        weight: Real::one(),
        active: true,
    });
    problem.add_constraint(Constraint {
        name: "x >= 5".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: x - Expr::int(5),
        weight: Real::one(),
        active: true,
    });

    let prepared = PreparedProblem::new(&problem);
    let report = certify_candidate_with_config(
        &prepared,
        &context_from_problem(&problem),
        CandidateCertificationConfig {
            min_precision: -128,
        },
    );

    assert_eq!(report.certified_satisfied_rows, 1);
    assert_eq!(report.certified_violation_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        report.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn residual_ball_certification_uses_hyperlimit_filter_boundary() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_constraint(Constraint::equality("x minus seven", x - Expr::int(7)));

    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    let report = certify_candidate_with_residual_balls(
        &prepared,
        &context,
        &[CandidateResidualBall {
            active_row: 0,
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.certified_violation_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::BallCertified {
            sign: hyperreal::RealSign::Positive
        }
    ));
}

#[test]
fn residual_ball_certification_rejects_negative_radius() {
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("zero", Expr::zero()));

    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    let report = certify_candidate_with_residual_balls(
        &prepared,
        &context,
        &[CandidateResidualBall {
            active_row: 0,
            radius: real(-1),
        }],
        hyperlimit::PredicatePolicy::default(),
    );

    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::InvalidBallRadius
    ));
}

#[test]
fn affine_interval_candidate_certifies_box_away_from_zero() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_constraint(Constraint::equality("x minus seven", x - Expr::int(7)));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_affine_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect("affine interval certification should be valid");

    assert_eq!(report.certified_violation_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::BallCertified {
            sign: hyperreal::RealSign::Positive
        }
    ));
}

#[test]
fn affine_interval_candidate_rejects_negative_variable_radius() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_constraint(Constraint::equality("x minus seven", x - Expr::int(7)));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let error = certify_affine_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(-1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect_err("negative variable radius must be rejected");

    assert_eq!(
        error,
        hypersolve::AffineIntervalError::NegativeVariableRadius {
            symbol: SymbolId(0)
        }
    );
}

#[test]
fn affine_krawczyk_box_certifies_unique_root_inside_exact_box() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_variable("y", real(2));
    problem.add_constraint(Constraint::equality(
        "x plus y minus five",
        x.clone() + y.clone() - Expr::int(5),
    ));
    problem.add_constraint(Constraint::equality(
        "x minus y minus one",
        x - y - Expr::int(1),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_affine_krawczyk_box(
        &prepared,
        &context,
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: real(1),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: Real::zero(),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(
        report.status,
        hypersolve::AffineKrawczykStatus::CertifiedUniqueRoot
    );
    assert_eq!(report.variable_count, 2);
    assert_eq!(report.equality_rows, 2);
    assert_eq!(report.steps[0].step, real(1));
    assert_eq!(report.steps[0].certified_root, real(3));
    assert_eq!(report.steps[1].step, Real::zero());
    assert_eq!(report.steps[1].certified_root, real(2));
}

#[test]
fn affine_krawczyk_box_reports_outside_and_singular_cases() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut outside = Problem::default();
    outside.add_variable("x", real(2));
    outside.add_variable("y", real(2));
    outside.add_constraint(Constraint::equality(
        "x plus y minus five",
        x.clone() + y.clone() - Expr::int(5),
    ));
    outside.add_constraint(Constraint::equality(
        "x minus y minus one",
        x.clone() - y.clone() - Expr::int(1),
    ));
    let outside_report = certify_affine_krawczyk_box(
        &PreparedProblem::new(&outside),
        &context_from_problem(&outside),
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: Real::zero(),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: Real::zero(),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        outside_report.status,
        hypersolve::AffineKrawczykStatus::RootOutsideBox {
            symbol: SymbolId(0)
        }
    );

    let mut singular = Problem::default();
    singular.add_variable("x", real(0));
    singular.add_variable("y", real(0));
    singular.add_constraint(Constraint::equality("x plus y", x.clone() + y.clone()));
    singular.add_constraint(Constraint::equality("duplicate x plus y", x + y));
    let singular_report = certify_affine_krawczyk_box(
        &PreparedProblem::new(&singular),
        &context_from_problem(&singular),
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: real(1),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: real(1),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(
        singular_report.status,
        hypersolve::AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot: 1 }
    );
}

#[test]
fn quadratic_interval_candidate_certifies_taylor_ball_away_from_zero() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "unit quadratic below zero",
        (x.clone() * x) - Expr::int(100),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_quadratic_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect("quadratic interval certification should be valid");

    assert_eq!(report.certified_violation_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::BallCertified {
            sign: hyperreal::RealSign::Negative
        }
    ));
}

#[test]
fn quadratic_interval_candidate_certifies_zero_radius_root() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality(
        "quadratic root",
        (x.clone() * x) - Expr::int(9),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_quadratic_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: Real::zero(),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect("zero-radius quadratic interval certification should be valid");

    assert_eq!(report.certified_satisfied_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::BallCertified {
            sign: hyperreal::RealSign::Zero
        }
    ));
}

#[test]
fn multivariate_quadratic_interval_candidate_certifies_cross_term_ball() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_variable("y", real(10));
    problem.add_constraint(Constraint::equality(
        "cross term positive",
        x.clone() * y.clone() - Expr::int(50),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_multivariate_quadratic_interval_candidate(
        &prepared,
        &context,
        &[
            VariableBall {
                symbol: SymbolId(0),
                radius: real(1),
            },
            VariableBall {
                symbol: SymbolId(1),
                radius: real(1),
            },
        ],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect("multivariate quadratic interval certification should be valid");

    assert_eq!(report.certified_violation_rows, 1);
    assert!(matches!(
        report.rows[0].status,
        CertifiedCandidateStatus::BallCertified {
            sign: hyperreal::RealSign::Positive
        }
    ));
}

#[test]
fn multivariate_quadratic_interval_candidate_rejects_missing_binding() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_variable("y", real(10));
    problem.add_constraint(Constraint::equality("cross term", x * y));
    let prepared = PreparedProblem::new(&problem);
    let mut context = hypersolve::EvaluationContext::default();
    context.bind(SymbolId(0), real(10));

    let error = certify_multivariate_quadratic_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect_err("cross-term interval certification needs both candidate centers");

    assert_eq!(
        error,
        hypersolve::QuadraticIntervalError::UnboundCandidateSymbol {
            symbol: SymbolId(1)
        }
    );
}

#[test]
fn quadratic_interval_candidate_rejects_invalid_inputs() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality(
        "quadratic root",
        (x.clone() * x) - Expr::int(9),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let negative_radius = certify_quadratic_interval_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(-1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect_err("negative variable radius must be rejected");
    assert_eq!(
        negative_radius,
        hypersolve::QuadraticIntervalError::NegativeVariableRadius {
            symbol: SymbolId(0)
        }
    );

    let missing_binding = certify_quadratic_interval_candidate(
        &prepared,
        &hypersolve::EvaluationContext::default(),
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        hyperlimit::PredicatePolicy::default(),
    )
    .expect_err("quadratic interval certification needs the candidate center");
    assert_eq!(
        missing_binding,
        hypersolve::QuadraticIntervalError::UnboundCandidateSymbol {
            symbol: SymbolId(0)
        }
    );
}

#[test]
fn direct_affine_solver_isolates_one_variable_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "2x - 10",
        x.clone() * Expr::int(2) - Expr::int(10),
    ));
    problem.add_constraint(Constraint::equality("coupled", x + y));

    let prepared = PreparedProblem::new(&problem);
    let solutions = solve_direct_affine_equalities(&prepared).unwrap();

    assert_eq!(solutions.len(), 1);
    assert_eq!(solutions[0].constraint_index, 0);
    assert_eq!(solutions[0].symbol, SymbolId(0));
    assert_eq!(solutions[0].value, real(5));
}

#[test]
fn interval_box_report_retains_payload_and_status() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(10));
    problem.add_constraint(Constraint::equality("x minus seven", x - Expr::int(7)));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    let report = certify_interval_box_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(1),
        }],
        IntervalBoxCertificationPackage::Affine,
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.status, IntervalBoxCertificationStatus::Violation);
    assert_eq!(report.variable_balls.len(), 1);
    assert!(report.certification.unwrap().has_certified_violation());

    let invalid = certify_interval_box_candidate(
        &prepared,
        &context,
        &[VariableBall {
            symbol: SymbolId(0),
            radius: real(-1),
        }],
        IntervalBoxCertificationPackage::Affine,
        hyperlimit::PredicatePolicy::default(),
    );
    assert_eq!(invalid.status, IntervalBoxCertificationStatus::InvalidInput);
    assert!(invalid.certification.is_none());
    assert!(invalid.message.is_some());
}

#[test]
fn direct_affine_system_solves_square_rows_and_replays_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "2x plus y minus five",
        x.clone() * Expr::int(2) + y.clone() - Expr::int(5),
    ));
    problem.add_constraint(Constraint::equality(
        "x minus y minus one",
        x - y - Expr::int(1),
    ));

    let report = solve_direct_affine_system(&PreparedProblem::new(&problem));

    assert_eq!(report.status, DirectAffineSystemStatus::Solved);
    assert!(report.solved());
    assert_eq!(report.variable_count, 2);
    assert_eq!(report.equality_rows, 2);
    assert_eq!(report.constraint_indices, vec![0, 1]);
    assert_eq!(report.assignments.len(), 2);
    assert_eq!(report.assignments[0].symbol, SymbolId(0));
    assert_eq!(report.assignments[0].value, real(2));
    assert_eq!(report.assignments[1].symbol, SymbolId(1));
    assert_eq!(report.assignments[1].value, real(1));

    let mut candidate = context_from_problem(&problem);
    for assignment in &report.assignments {
        candidate.bind(assignment.symbol, assignment.value.clone());
    }
    let certification = certify_candidate(&PreparedProblem::new(&problem), &candidate);
    assert!(certification.all_satisfied());

    let mut underdetermined = Problem::default();
    underdetermined.add_variable("x", real(0));
    underdetermined.add_variable("y", real(0));
    underdetermined.add_constraint(Constraint::equality(
        "only one row",
        Expr::symbol(SymbolId(0), "x") - Expr::int(2),
    ));
    let shape = solve_direct_affine_system(&PreparedProblem::new(&underdetermined));
    assert_eq!(
        shape.status,
        DirectAffineSystemStatus::ShapeMismatch {
            variables: 2,
            equality_rows: 1
        }
    );

    let mut singular = Problem::default();
    let sx = Expr::symbol(SymbolId(0), "x");
    let sy = Expr::symbol(SymbolId(1), "y");
    singular.add_variable("x", real(0));
    singular.add_variable("y", real(0));
    singular.add_constraint(Constraint::equality("x plus y", sx.clone() + sy.clone()));
    singular.add_constraint(Constraint::equality("duplicate", sx + sy));
    let singular_report = solve_direct_affine_system(&PreparedProblem::new(&singular));
    assert_eq!(
        singular_report.status,
        DirectAffineSystemStatus::SingularOrUnsupportedPivot { pivot: 1 }
    );
}

#[test]
fn exact_affine_rank_reports_dof_and_inconsistency_without_lossy_rank_hints() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut underconstrained = Problem::default();
    underconstrained.add_variable("x", real(0));
    underconstrained.add_variable("y", real(0));
    underconstrained.add_constraint(Constraint::equality("x", x.clone()));
    let rank = analyze_exact_affine_rank(&PreparedProblem::new(&underconstrained), -64);
    assert_eq!(rank.status, ExactAffineRankStatus::Certified);
    assert_eq!(rank.coefficient_rank, Some(1));
    assert_eq!(rank.augmented_rank, Some(1));
    assert_eq!(rank.degrees_of_freedom, Some(1));

    let mut inconsistent = Problem::default();
    inconsistent.add_variable("x", real(0));
    inconsistent.add_variable("y", real(0));
    inconsistent.add_constraint(Constraint::equality("x", x.clone()));
    inconsistent.add_constraint(Constraint::equality("x plus one", x.clone() + Expr::int(1)));
    let rank = analyze_exact_affine_rank(&PreparedProblem::new(&inconsistent), -64);
    assert_eq!(rank.status, ExactAffineRankStatus::Inconsistent);
    assert_eq!(rank.coefficient_rank, Some(1));
    assert_eq!(rank.augmented_rank, Some(2));
    assert_eq!(rank.degrees_of_freedom, Some(1));

    let mut square = Problem::default();
    square.add_variable("x", real(0));
    square.add_variable("y", real(0));
    square.add_constraint(Constraint::equality("x", x));
    square.add_constraint(Constraint::equality("y", y));
    let rank = analyze_exact_affine_rank(&PreparedProblem::new(&square), -64);
    assert_eq!(rank.status, ExactAffineRankStatus::Certified);
    assert_eq!(rank.coefficient_rank, Some(2));
    assert_eq!(rank.augmented_rank, Some(2));
    assert_eq!(rank.degrees_of_freedom, Some(0));
}

#[test]
fn exact_affine_rank_reports_unsupported_nonlinear_rows_and_skips_inequalities() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("nonlinear", x.clone() * x.clone()));
    let mut inequality = Constraint::equality("bound", x - Expr::int(1));
    inequality.kind = ConstraintKind::LessOrEqual;
    problem.add_constraint(inequality);

    let rank = analyze_exact_affine_rank(&PreparedProblem::new(&problem), -64);

    assert_eq!(rank.status, ExactAffineRankStatus::UnsupportedNonAffineRows);
    assert_eq!(rank.unsupported_rows, vec![0]);
    assert_eq!(rank.skipped_non_equality_rows, 1);
    assert_eq!(rank.coefficient_rank, None);
}

#[test]
fn equality_substitution_finds_unit_difference_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality("x - y + 3", x - y + Expr::int(3)));

    let prepared = PreparedProblem::new(&problem);
    let substitutions = find_equality_substitutions(&prepared).unwrap();

    assert_eq!(substitutions.len(), 1);
    assert_eq!(substitutions[0].left, SymbolId(0));
    assert_eq!(substitutions[0].right, SymbolId(1));
    assert_eq!(substitutions[0].offset, real(-3));
}

#[test]
fn equality_substitution_updates_candidate_context_exactly() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(10));
    problem.add_constraint(Constraint::equality("x - y + 3", x - y + Expr::int(3)));
    let prepared = PreparedProblem::new(&problem);
    let substitutions = find_equality_substitutions(&prepared).unwrap();
    let mut context = context_from_problem(&problem);

    let applied = apply_equality_substitutions(&mut context, &substitutions).unwrap();

    assert_eq!(applied, 1);
    assert_eq!(context.bindings().get(&SymbolId(0)), Some(&real(7)));
}

#[test]
fn equality_substitution_validation_reports_cycles_and_conflicts_exactly() {
    let substitutions = vec![
        hypersolve::EqualitySubstitution {
            constraint_index: 0,
            left: SymbolId(0),
            right: SymbolId(1),
            offset: real(2),
        },
        hypersolve::EqualitySubstitution {
            constraint_index: 1,
            left: SymbolId(1),
            right: SymbolId(0),
            offset: real(-2),
        },
        hypersolve::EqualitySubstitution {
            constraint_index: 2,
            left: SymbolId(2),
            right: SymbolId(2),
            offset: real(1),
        },
        hypersolve::EqualitySubstitution {
            constraint_index: 3,
            left: SymbolId(0),
            right: SymbolId(2),
            offset: real(0),
        },
    ];

    let report = validate_equality_substitutions(&substitutions);

    assert!(!report.is_acyclic_rewrite_graph());
    assert!(report.has_inconsistency());
    assert!(report.problems.iter().any(|problem| matches!(
        problem,
        hypersolve::EqualitySubstitutionProblem::DirectedCycle {
            symbols,
            net_offset,
            consistent: true,
        } if symbols == &vec![SymbolId(0), SymbolId(1)] && net_offset == &Real::zero()
    )));
    assert!(report.problems.iter().any(|problem| matches!(
        problem,
        hypersolve::EqualitySubstitutionProblem::SelfSubstitution {
            symbol: SymbolId(2),
            offset,
            consistent: false,
            ..
        } if offset == &real(1)
    )));
    assert!(report.problems.iter().any(|problem| matches!(
        problem,
        hypersolve::EqualitySubstitutionProblem::DuplicateLeft {
            left: SymbolId(0),
            same_rewrite: false,
            ..
        }
    )));
}

#[test]
fn equality_substitution_classes_preserve_offsets_to_representative() {
    let substitutions = vec![
        hypersolve::EqualitySubstitution {
            constraint_index: 0,
            left: SymbolId(2),
            right: SymbolId(1),
            offset: real(5),
        },
        hypersolve::EqualitySubstitution {
            constraint_index: 1,
            left: SymbolId(1),
            right: SymbolId(0),
            offset: real(-3),
        },
    ];

    let classes = build_equality_substitution_classes(&substitutions).unwrap();

    assert_eq!(classes.len(), 1);
    assert_eq!(classes[0].representative, SymbolId(0));
    assert_eq!(classes[0].members.len(), 3);
    assert_eq!(classes[0].members[0].symbol, SymbolId(0));
    assert_eq!(classes[0].members[0].offset_from_representative, real(0));
    assert_eq!(classes[0].members[1].symbol, SymbolId(1));
    assert_eq!(classes[0].members[1].offset_from_representative, real(-3));
    assert_eq!(classes[0].members[2].symbol, SymbolId(2));
    assert_eq!(classes[0].members[2].offset_from_representative, real(2));
}

#[test]
fn equality_substitution_class_application_populates_candidate_or_reports_conflict() {
    let substitutions = vec![
        hypersolve::EqualitySubstitution {
            constraint_index: 0,
            left: SymbolId(1),
            right: SymbolId(0),
            offset: real(3),
        },
        hypersolve::EqualitySubstitution {
            constraint_index: 1,
            left: SymbolId(2),
            right: SymbolId(1),
            offset: real(-5),
        },
    ];
    let classes = build_equality_substitution_classes(&substitutions).unwrap();

    let mut context = hypersolve::EvaluationContext::default();
    context.bind(SymbolId(2), real(9));
    let report = apply_equality_substitution_classes(&mut context, &classes);

    assert!(report.all_consistent());
    assert_eq!(report.applied_bindings, 3);
    assert_eq!(report.skipped_classes, 0);
    assert_eq!(context.bindings().get(&SymbolId(0)), Some(&real(11)));
    assert_eq!(context.bindings().get(&SymbolId(1)), Some(&real(14)));
    assert_eq!(context.bindings().get(&SymbolId(2)), Some(&real(9)));
    assert!(matches!(
        report.rows[0].status,
        hypersolve::EqualitySubstitutionClassApplicationStatus::Applied {
            anchor_symbol: SymbolId(2)
        }
    ));

    let mut missing = hypersolve::EvaluationContext::default();
    let missing_report = apply_equality_substitution_classes(&mut missing, &classes);
    assert_eq!(missing_report.applied_bindings, 0);
    assert_eq!(missing_report.skipped_classes, 1);
    assert!(matches!(
        missing_report.rows[0].status,
        hypersolve::EqualitySubstitutionClassApplicationStatus::MissingBoundMember
    ));

    let mut conflict = hypersolve::EvaluationContext::default();
    conflict.bind(SymbolId(0), real(11));
    conflict.bind(SymbolId(1), real(13));
    let conflict_report = apply_equality_substitution_classes(&mut conflict, &classes);
    assert!(!conflict_report.all_consistent());
    assert_eq!(conflict_report.applied_bindings, 0);
    assert_eq!(conflict_report.inconsistent_classes, 1);
    assert!(matches!(
        &conflict_report.rows[0].status,
        hypersolve::EqualitySubstitutionClassApplicationStatus::InconsistentBoundMember {
            symbol: SymbolId(1),
            expected,
            actual,
        } if expected == &real(14) && actual == &real(13)
    ));
}

#[test]
fn equality_substitution_elimination_carries_exact_offsets_into_affine_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let z = Expr::symbol(SymbolId(2), "z");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_variable("z", real(0));
    problem.add_constraint(Constraint::equality(
        "x - y + 3",
        x.clone() - y.clone() + Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "z - x - 2",
        z.clone() - x.clone() - Expr::int(2),
    ));
    problem.add_constraint(Constraint::equality(
        "2z + y + 5",
        z * Expr::int(2) + y + Expr::int(5),
    ));

    let prepared = PreparedProblem::new(&problem);
    let substitutions = find_equality_substitutions(&prepared).unwrap();
    let classes = build_equality_substitution_classes(&substitutions).unwrap();
    let report = eliminate_affine_rows_with_substitution_classes(&prepared, &classes);

    assert_eq!(report.affine_rows_considered, 3);
    assert_eq!(report.reduced_variable_rows, 1);
    assert_eq!(report.reduced_zero_rows, 2);
    assert_eq!(report.reduced_contradiction_rows, 0);
    assert_eq!(report.reduced_unknown_constant_rows, 0);
    assert_eq!(report.rows.len(), 3);
    assert_eq!(classes[0].representative, SymbolId(0));

    let reduced = &report.rows[2];
    assert_eq!(reduced.constraint_index, 2);
    assert_eq!(reduced.constant, real(12));
    assert_eq!(reduced.coefficients, vec![(SymbolId(0), real(3))]);
}

#[test]
fn equality_substitution_elimination_classifies_reduced_constant_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "y - x - 3",
        y.clone() - x.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "y - x - 3 again",
        y.clone() - x.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "y - x - 4 contradiction",
        y - x - Expr::int(4),
    ));

    let prepared = PreparedProblem::new(&problem);
    let substitutions = vec![hypersolve::EqualitySubstitution {
        constraint_index: 0,
        left: SymbolId(1),
        right: SymbolId(0),
        offset: real(3),
    }];
    let classes = build_equality_substitution_classes(&substitutions).unwrap();
    let report = eliminate_affine_rows_with_substitution_classes(&prepared, &classes);

    assert_eq!(report.affine_rows_considered, 3);
    assert_eq!(report.reduced_variable_rows, 0);
    assert_eq!(report.reduced_zero_rows, 2);
    assert_eq!(report.reduced_contradiction_rows, 1);
    assert_eq!(report.reduced_unknown_constant_rows, 0);
    assert_eq!(report.rows[2].constant, real(-1));
    assert!(report.rows[2].coefficients.is_empty());
}

#[test]
fn prepared_problem_evaluates_through_source_problem_without_float_topology() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality("x minus three", x - Expr::int(3)));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);

    let residuals = prepared.evaluate_residuals(&context).unwrap();

    assert_eq!(residuals.len(), 1);
    assert_eq!(residuals[0].value, Real::zero());
    assert_eq!(residuals[0].sign, Some(hyperreal::RealSign::Zero));
}

#[test]
fn prepared_symbolic_jacobian_consumes_structural_sparsity() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_variable("y", real(11));
    problem.add_constraint(Constraint::equality(
        "x-only affine row",
        x.clone() * Expr::int(3) - Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality("coupled row", x.clone() * y.clone()));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);

    let plain = symbolic_jacobian(&problem, &context).unwrap();
    let sparse = symbolic_jacobian_prepared(&prepared, &context).unwrap();

    assert_eq!(sparse, plain);
    assert_eq!(prepared.jacobian_sparsity()[0], vec![true, false]);
    assert_eq!(sparse[0][1], 0.0);
    assert_eq!(sparse[0][0], 3.0);
}

#[test]
fn dense_linear_adapter_reports_lossy_pivot_diagnostics() {
    let backend = DenseLinearBackend;
    let jacobian = vec![vec![0.0, 2.0], vec![3.0, 0.0]];
    let residuals = vec![-4.0, -9.0];

    let (step, report) = backend
        .solve_damped_normal(&jacobian, &residuals, 0.0)
        .expect("nonsingular dense adapter solve should succeed");

    assert_eq!(report.adapter, LinearAdapterKind::DenseF64NormalEquations);
    assert_eq!(report.precision(), LinearAdapterPrecision::LossyF64);
    assert!(report.lossy);
    assert!(report.is_lossy());
    assert_eq!(report.rank_hint, Some(2));
    assert_eq!(report.pivot_count, 2);
    assert_eq!(report.row_swaps, 0);
    assert!(report.min_abs_pivot.unwrap() > 0.0);
    assert!(report.max_abs_pivot.unwrap() >= report.min_abs_pivot.unwrap());
    assert!((step[0] - 3.0).abs() < 1.0e-9);
    assert!((step[1] - 2.0).abs() < 1.0e-9);
}

#[test]
fn candidate_domain_certification_reports_valid_invalid_and_nested_domains() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(4));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality("sqrt valid", x.clone().sqrt()));
    problem.add_constraint(Constraint::equality("sqrt invalid", (-x.clone()).sqrt()));
    problem.add_constraint(Constraint::equality(
        "division invalid",
        x.clone() / y.clone(),
    ));
    problem.add_constraint(Constraint::equality(
        "nested log division",
        (x.clone() / Expr::int(2)).log10(),
    ));

    let report = certify_candidate_domains(
        &problem,
        &context_from_problem(&problem),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.active_constraint_count, 4);
    assert_eq!(report.checks.len(), 5);
    assert_eq!(report.certified_valid_checks, 3);
    assert_eq!(report.certified_invalid_checks, 2);
    assert!(report.has_certified_invalid_domain());
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::SqrtOperandNonNegative
            && check.constraint_name == "sqrt invalid"
            && check.status == DomainCheckStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::DivisionDenominatorNonZero
            && check.constraint_name == "division invalid"
            && check.status == DomainCheckStatus::CertifiedInvalid
    }));
}

#[test]
fn candidate_domain_certification_reports_negative_power_and_unbound_symbols() {
    let x = Expr::symbol(SymbolId(0), "x");
    let z = Expr::symbol(SymbolId(9), "z");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("inverse invalid", x.powi(-1)));
    problem.add_constraint(Constraint::equality("unbound sqrt", z.sqrt()));

    let report = certify_candidate_domains(
        &problem,
        &context_from_problem(&problem),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.checks.len(), 2);
    assert_eq!(report.certified_invalid_checks, 1);
    assert_eq!(report.evaluation_failure_checks, 1);
    assert!(matches!(
        report.checks[0].status,
        DomainCheckStatus::CertifiedInvalid
    ));
    assert!(matches!(
        report.checks[1].status,
        DomainCheckStatus::UnboundSymbol {
            symbol: SymbolId(9)
        }
    ));
}

#[test]
fn candidate_domain_certification_reports_inverse_function_boundaries() {
    let ln_value = Expr::symbol(SymbolId(0), "ln_value");
    let asin_value = Expr::symbol(SymbolId(1), "asin_value");
    let acos_value = Expr::symbol(SymbolId(2), "acos_value");
    let acosh_value = Expr::symbol(SymbolId(3), "acosh_value");
    let atanh_value = Expr::symbol(SymbolId(4), "atanh_value");
    let mut problem = Problem::default();
    problem.add_variable("ln_value", real(0));
    problem.add_variable("asin_value", real(2));
    problem.add_variable("acos_value", real(-1));
    problem.add_variable("acosh_value", real(1));
    problem.add_variable("atanh_value", real(1));
    problem.add_constraint(Constraint::equality("ln invalid", ln_value.ln()));
    problem.add_constraint(Constraint::equality("asin invalid", asin_value.asin()));
    problem.add_constraint(Constraint::equality(
        "acos valid boundary",
        acos_value.acos(),
    ));
    problem.add_constraint(Constraint::equality(
        "acosh valid boundary",
        acosh_value.acosh(),
    ));
    problem.add_constraint(Constraint::equality(
        "atanh invalid boundary",
        atanh_value.atanh(),
    ));

    let report = certify_candidate_domains(
        &problem,
        &context_from_problem(&problem),
        hyperlimit::PredicatePolicy::default(),
    );

    assert_eq!(report.checks.len(), 5);
    assert_eq!(report.certified_valid_checks, 2);
    assert_eq!(report.certified_invalid_checks, 3);
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::LnOperandPositive
            && check.status == DomainCheckStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::AsinOperandInClosedUnitInterval
            && check.status == DomainCheckStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::AcosOperandInClosedUnitInterval
            && check.status == DomainCheckStatus::CertifiedValid
    }));
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::AcoshOperandAtLeastOne
            && check.status == DomainCheckStatus::CertifiedValid
    }));
    assert!(report.checks.iter().any(|check| {
        check.kind == DomainCheckKind::AtanhOperandInOpenUnitInterval
            && check.status == DomainCheckStatus::CertifiedInvalid
    }));
}
