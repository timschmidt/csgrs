use hyperreal::{Rational, Real, SymbolicDependencyMask};
use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_prepared};
use hypersolve::{
    ActiveSetAffineRegenerationStatus, BatchCandidateStatus, BatchPredicateScheduleConfig,
    BatchPredicateScheduleError, CandidateCertificationConfig, CandidateResidualBall,
    CertifiedCandidateStatus, Constraint, ConstraintKind, ConvergenceReason, DenseLinearBackend,
    DirectAffineSystemStatus, DomainCheckKind, DomainCheckStatus, DraggedParameterWeight,
    ExactAffineRankStatus, ExactBranchStatus, Expr, ExprDegree, FailedConstraintRemovalStatus,
    FailedConstraintStatus, IntervalBoxCertificationPackage, IntervalBoxCertificationStatus,
    LinearAdapterKind, LinearAdapterPrecision, LinearBackend, MultivariateQuadraticKrawczykStatus,
    PreparedProblem, PreparedSolverBlock, Problem, ProposalEngineKind, ProposalEnginePrecision,
    RootIsolationStatus, RootMultiplicityStatus, SketchArcEndpoint, SketchArcLengthSweep,
    SketchArcPointSweep, SketchArcTangencyBranch, SketchConstraintKind,
    SketchConstructionCertificateStatus, SketchDegeneracyKind, SketchDegeneracyStatus,
    SketchEntityDomain, SketchEntityDomainKind, SketchEntityDomainStatus, SketchEntityHandle,
    SketchEntityKind, SketchFailedConstraintStatus, SketchGeneratedRowStatus, SketchLineEndpoint,
    SketchParameterDomain, SketchParameterDomainKind, SketchParameterDomainStatus,
    SketchResidualFormKind, SketchResidualFormRole, SketchResidualFormsStatus,
    SketchResidualStrategy, SketchRoundTripMetadata, SketchRoundTripRole, SketchSolveProblem,
    SketchTangentOrientation, SketchUnitToleranceStatus, SketchWorkplaneFrameStatus,
    SolverBlockRowKind, SolverConfig, SolverPoint2, SolverState, SparseResidualBatchStatus,
    SparseResidualTerm, SymbolId, VariableBall, analyze_exact_affine_rank,
    apply_equality_substitution_classes, apply_equality_substitutions,
    audit_sketch_unit_tolerances, build_equality_substitution_classes,
    build_sketch_workplane_frame, certify_affine_interval_candidate, certify_affine_krawczyk_box,
    certify_candidate, certify_candidate_batch, certify_candidate_domains,
    certify_candidate_with_config, certify_candidate_with_residual_balls,
    certify_direct_univariate_quadratic_roots, certify_interval_box_candidate,
    certify_multivariate_quadratic_interval_candidate, certify_multivariate_quadratic_krawczyk_box,
    certify_quadratic_interval_candidate, certify_sketch_construction,
    certify_univariate_quadratic_alpha, certify_univariate_quadratic_krawczyk_box,
    context_from_problem, diagnose_failed_constraints,
    diagnose_failed_constraints_from_certification, diagnose_sketch_failed_constraints,
    eliminate_affine_rows_with_substitution_classes,
    enumerate_direct_univariate_quadratic_branches, evaluate_residuals, facts_depend_on_symbol,
    find_equality_substitutions, isolate_univariate_polynomial_roots,
    lift_sketch_point2_to_workplane3, point_coincidence_equations, preflight_sketch_degeneracies,
    preflight_sketch_entity_domains, preflight_sketch_parameter_domains,
    prepare_sparse_linear_residual_system, project_sketch_point3_to_workplane2,
    regenerate_active_set_affine_candidate, replay_sketch_compatibility_fixture,
    replay_sparse_linear_residual_batch, report_lossy_adapter_only_candidate,
    schedule_candidate_batch_predicates, search_failed_constraint_minimal_removals,
    search_failed_constraint_pair_removals, search_failed_constraint_set_removals,
    search_failed_constraint_single_removals, sketch_angle_builders, sketch_compatibility_fixtures,
    sketch_distance_builders, sketch_incidence_builders, sketch_objective_builders,
    sketch_orientation_builders, sketch_range_builders, sketch_symmetry_builders,
    sketch_tangency_builders, solve_damped_least_squares, solve_direct_affine_equalities,
    solve_direct_affine_system, solve_direct_univariate_quadratic_equalities,
    squared_distance_equation, tangent_parallel_equation, tangent_same_direction_constraint,
    validate_equality_substitutions,
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
fn sketch_point_line_distance_constraints_retain_signed_distance_forms() {
    let mut sketch = SketchSolveProblem::new();
    let start = sketch.add_point2d("start", real(0), real(0));
    let end = sketch.add_point2d("end", real(5), real(0));
    let above = sketch.add_point2d("above", real(2), real(3));
    let line = sketch.add_line_segment2("line", start, end);
    let distance = sketch.add_distance("distance", real(3));
    let relation = sketch_distance_builders::point_line_distance2(
        &mut sketch,
        "point line distance",
        above,
        line,
        distance,
    );

    let forms = sketch.residual_forms_for_constraint(relation.handle);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::SquaredPointLineDistancePolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::SquaredPointLineDistance)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::PointLineSignedDistancePositiveProposal
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ProposalOnly);
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::PointLineSignedDistanceNegativeProposal
    );
    assert_eq!(forms.forms[2].role, SketchResidualFormRole::ProposalOnly);

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
        real(-30)
    );
    assert_eq!(
        forms.forms[2]
            .residual
            .eval_real(context.bindings())
            .unwrap(),
        Real::zero()
    );
}

#[test]
fn sketch_point_on_circle_constraints_retain_exact_and_radial_forms() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let point = sketch.add_point2d("point", real(3), real(4));
    let radius = sketch.add_distance("radius", real(5));
    let circle = sketch.add_circle2("circle", center, radius);
    let relation =
        sketch_incidence_builders::point_on_circle(&mut sketch, "point on circle", point, circle);

    let forms = sketch.residual_forms_for_constraint(relation.handle);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::SquaredCircleIncidencePolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::SquaredIncidence)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::CircleRadialDistanceProposal
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
fn sketch_point_on_line_replays_unnormalized_collinearity_rows() {
    let mut sketch = SketchSolveProblem::new();
    let start = sketch.add_point2d("start", real(1), real(2));
    let end = sketch.add_point2d("end", real(7), real(5));
    let line = sketch.add_line_segment2("line", start, end);
    let point = sketch.add_point2d("point", real(5), real(4));
    let off_line = sketch.add_point2d("off line", real(5), real(5));
    let distance = sketch.add_distance("not a line", real(3));
    let valid =
        sketch_incidence_builders::point_on_line2(&mut sketch, "point on line", point, line);
    sketch.add_point_on_line2("off line", off_line, line);
    let wrong = sketch.add_point_on_line2("wrong line kind", point, distance);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Incidence);
    assert_eq!(valid.strategy, SketchResidualStrategy::PointLineIncidence);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::PointLineIncidence)
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: distance,
            expected: "2D line segment",
        }
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 1);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::PointLineIncidencePolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::PointLineIncidence)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_point_on_cubic_constraints_retain_exact_bernstein_rows() {
    let mut sketch = SketchSolveProblem::new();
    let p0 = sketch.add_point2d("p0", real(0), real(0));
    let p1 = sketch.add_point2d("p1", real(2), real(0));
    let p2 = sketch.add_point2d("p2", real(4), real(0));
    let p3 = sketch.add_point2d("p3", real(6), real(0));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let point = sketch.add_point2d("point", real(3), real(0));
    let parameter = sketch.add_parameter(
        "t",
        Real::new(Rational::fraction(1, 2).expect("nonzero denominator")),
    );
    sketch.add_parameter_domain(
        parameter,
        SketchParameterDomain::Bounded {
            lower: Some(real(0)),
            upper: Some(real(1)),
        },
    );
    let relation = sketch_incidence_builders::point_on_cubic2(
        &mut sketch,
        "point on cubic",
        point,
        cubic,
        parameter,
    );

    let lowered = sketch.lower_to_problem();
    let forms = sketch.residual_forms_for_constraint(relation.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(
        relation.family,
        hypersolve::SketchConstraintFamily::Incidence
    );
    assert_eq!(
        relation.strategy,
        SketchResidualStrategy::CubicBezierIncidence
    );
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicBezierIncidence)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert!(forms.forms.iter().all(|form| {
        form.kind == SketchResidualFormKind::CubicBezierIncidencePolynomial
            && form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::CubicBezierIncidence)
    }));
    for form in &forms.forms {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
}

#[test]
fn sketch_projected_point_on_cubic_replays_workplane_and_bernstein_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point2d("p0", real(0), real(0));
    let p1 = sketch.add_point2d("p1", real(2), real(0));
    let p2 = sketch.add_point2d("p2", real(4), real(0));
    let p3 = sketch.add_point2d("p3", real(6), real(0));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let point = sketch.add_point3d("point", real(4), real(2), real(99));
    let off_curve = sketch.add_point3d("off curve", real(4), real(3), real(99));
    let parameter = sketch.add_parameter(
        "t",
        Real::new(Rational::fraction(1, 2).expect("nonzero denominator")),
    );
    let valid = sketch_incidence_builders::projected_point_on_cubic3(
        &mut sketch,
        "projected point on cubic",
        workplane,
        point,
        cubic,
        parameter,
    );
    sketch.add_projected_point_on_cubic3(
        "off projected cubic",
        workplane,
        off_curve,
        cubic,
        parameter,
    );
    let wrong = sketch.add_projected_point_on_cubic3(
        "wrong projected cubic",
        workplane,
        point,
        point,
        parameter,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Incidence);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedCubicBezierIncidence
    );

    let lowered = sketch.lower_to_problem();
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(lowered.rows.len(), 7);
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point,
            expected: "2D cubic Bezier",
        }
    );
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..3].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedCubicBezierIncidence)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(
        certification.rows[..3]
            .iter()
            .all(|row| { matches!(row.status, CertifiedCandidateStatus::CertifiedZero { .. }) })
    );
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert!(forms.forms[1..].iter().all(|form| {
        form.kind == SketchResidualFormKind::ProjectedCubicBezierIncidencePolynomial
            && form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::ProjectedCubicBezierIncidence)
    }));
    for form in &forms.forms {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_cubic_line_tangent_lowers_exact_derivative_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let p0 = sketch.add_point2d("p0", real(0), real(0));
    let p1 = sketch.add_point2d("p1", real(1), real(1));
    let p2 = sketch.add_point2d("p2", real(2), real(2));
    let p3 = sketch.add_point2d("p3", real(3), real(3));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let parameter = sketch.add_parameter("t", real(1));
    let tangent_end = sketch.add_point2d("tangent end", real(4), real(4));
    let tangent = sketch.add_line_segment2("tangent", p3, tangent_end);
    let opposite_end = sketch.add_point2d("opposite end", real(2), real(2));
    let opposite = sketch.add_line_segment2("opposite", p3, opposite_end);
    let valid = sketch_tangency_builders::cubic_line_tangent2(
        &mut sketch,
        "cubic tangent",
        cubic,
        parameter,
        tangent,
        SketchLineEndpoint::Start,
    );
    sketch.add_cubic_line_tangent2(
        "opposite cubic tangent",
        cubic,
        parameter,
        opposite,
        SketchLineEndpoint::Start,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(valid.strategy, SketchResidualStrategy::CubicLineTangent);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 8);
    assert_eq!(lowered.rows.len(), 8);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicLineTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[7].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 4);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::CubicLineTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::CubicLineTangentCrossProductPredicate
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::CubicLineTangentDotProductPredicate
    );
    for form in &forms.forms[..3] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[3]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_projected_cubic_line_tangent_replays_workplane_endpoint_and_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point2d("p0", real(0), real(0));
    let p1 = sketch.add_point2d("p1", real(2), real(0));
    let p2 = sketch.add_point2d("p2", real(4), real(0));
    let p3 = sketch.add_point2d("p3", real(6), real(0));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let parameter = sketch.add_parameter(
        "t",
        Real::new(Rational::fraction(1, 2).expect("nonzero denominator")),
    );
    let line_start = sketch.add_point3d("line start", real(4), real(2), real(99));
    let line_end = sketch.add_point3d("line end", real(10), real(2), real(-7));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let reversed_end = sketch.add_point3d("reversed end", real(-2), real(2), real(8));
    let reversed = sketch.add_line_segment3("reversed", line_start, reversed_end);
    let valid = sketch_tangency_builders::projected_cubic_line_tangent3(
        &mut sketch,
        "projected cubic line tangent",
        workplane,
        cubic,
        parameter,
        line,
        SketchLineEndpoint::Start,
    );
    sketch.add_projected_cubic_line_tangent3(
        "reversed projected cubic line tangent",
        workplane,
        cubic,
        parameter,
        reversed,
        SketchLineEndpoint::Start,
    );
    let wrong = sketch.add_projected_cubic_line_tangent3(
        "wrong projected cubic line tangent kind",
        workplane,
        cubic,
        parameter,
        p3,
        SketchLineEndpoint::Start,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedCubicLineTangent
    );

    let lowered = sketch.lower_to_problem();
    let context = context_from_problem(&lowered.problem);
    let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
    let forms = sketch.residual_forms_for_constraint(valid.handle);

    assert_eq!(lowered.problem.constraints.len(), 10);
    assert_eq!(lowered.rows.len(), 11);
    assert!(lowered.rows[..10].iter().all(|row| {
        (row.strategy == Some(SketchResidualStrategy::ProjectedCubicLineTangent)
            || row.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion))
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p3,
            expected: "3D line segment",
        }
    );
    assert!(certification.rows[..5].iter().all(|row| matches!(
        row.status,
        CertifiedCandidateStatus::CertifiedZero { .. }
            | CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    )));
    assert!(matches!(
        certification.rows[9].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedCubicLineTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ProjectedCubicLineTangentCrossProductPredicate
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedCubicLineTangentDotProductPredicate
    );
    assert!(forms.forms.iter().all(|form| {
        form.role == SketchResidualFormRole::ExactProof
            && (form.strategy == Some(SketchResidualStrategy::ProjectedCubicLineTangent)
                || form.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion))
    }));
    for form in &forms.forms[..4] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[4]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_cubic_cubic_tangent_lowers_exact_derivative_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(1), real(1));
    let a2 = sketch.add_point2d("a2", real(2), real(2));
    let join = sketch.add_point2d("join", real(3), real(3));
    let b1 = sketch.add_point2d("b1", real(4), real(4));
    let b2 = sketch.add_point2d("b2", real(5), real(5));
    let b3 = sketch.add_point2d("b3", real(6), real(6));
    let reverse1 = sketch.add_point2d("reverse1", real(2), real(2));
    let reverse2 = sketch.add_point2d("reverse2", real(1), real(1));
    let reverse3 = sketch.add_point2d("reverse3", real(0), real(0));
    let first = sketch.add_cubic2("first", a0, a1, a2, join);
    let second = sketch.add_cubic2("second", join, b1, b2, b3);
    let opposite = sketch.add_cubic2("opposite", join, reverse1, reverse2, reverse3);
    let t_end = sketch.add_parameter("t_end", real(1));
    let t_start = sketch.add_parameter("t_start", real(0));
    let valid = sketch_tangency_builders::cubic_cubic_tangent2(
        &mut sketch,
        "cubic cubic tangent",
        first,
        t_end,
        second,
        t_start,
    );
    sketch.add_cubic_cubic_tangent2("opposite cubic tangent", first, t_end, opposite, t_start);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(valid.strategy, SketchResidualStrategy::CubicCubicTangent);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 8);
    assert_eq!(lowered.rows.len(), 8);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicCubicTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[7].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 4);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::CubicCubicTangentPointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::CubicCubicTangentCrossProductPredicate
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::CubicCubicTangentDotProductPredicate
    );
    for form in &forms.forms[..3] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[3]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_arc_cubic_tangent_lowers_exact_derivative_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let p1 = sketch.add_point2d("p1", real(5), real(2));
    let p2 = sketch.add_point2d("p2", real(5), real(4));
    let p3 = sketch.add_point2d("p3", real(5), real(6));
    let bad_p1 = sketch.add_point2d("bad p1", real(5), real(-2));
    let bad_p2 = sketch.add_point2d("bad p2", real(5), real(-4));
    let bad_p3 = sketch.add_point2d("bad p3", real(5), real(-6));
    let cubic = sketch.add_cubic2("cubic", start, p1, p2, p3);
    let bad_cubic = sketch.add_cubic2("bad cubic", start, bad_p1, bad_p2, bad_p3);
    let t0 = sketch.add_parameter("t0", real(0));
    let valid = sketch_tangency_builders::arc_cubic_tangent2(
        &mut sketch,
        "arc cubic tangent",
        arc,
        SketchArcEndpoint::Start,
        cubic,
        t0,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_cubic_tangent2(
        "bad arc cubic tangent",
        arc,
        SketchArcEndpoint::Start,
        bad_cubic,
        t0,
        SketchTangentOrientation::CounterClockwise,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(valid.strategy, SketchResidualStrategy::ArcCubicTangent);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 10);
    assert_eq!(lowered.rows.len(), 10);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ArcCubicTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    for row in &certification.rows[..4] {
        assert!(matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ));
    }
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[9].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::ArcCubicTangentEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ArcCubicTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ArcCubicTangentRadiusPerpendicularPolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ArcCubicTangentOrientationPredicate
    );
    for form in &forms.forms[..4] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[4]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_cubic_cubic_g2_lowers_exact_curvature_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(1), real(0));
    let a2 = sketch.add_point2d("a2", real(2), real(0));
    let join = sketch.add_point2d("join", real(3), real(0));
    let b1 = sketch.add_point2d("b1", real(4), real(0));
    let b2 = sketch.add_point2d("b2", real(5), real(0));
    let b3 = sketch.add_point2d("b3", real(6), real(0));
    let bad_b2 = sketch.add_point2d("bad b2", real(5), real(1));
    let first = sketch.add_cubic2("first", a0, a1, a2, join);
    let second = sketch.add_cubic2("second", join, b1, b2, b3);
    let bad_second = sketch.add_cubic2("bad second", join, b1, bad_b2, b3);
    let t_end = sketch.add_parameter("t_end", real(1));
    let t_start = sketch.add_parameter("t_start", real(0));
    let valid = sketch_tangency_builders::cubic_cubic_g2_continuity2(
        &mut sketch,
        "cubic cubic g2",
        first,
        t_end,
        second,
        t_start,
    );
    sketch.add_cubic_cubic_g2_continuity2("bad cubic g2", first, t_end, bad_second, t_start);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::CubicCubicG2Continuity
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 12);
    assert_eq!(lowered.rows.len(), 12);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicCubicG2Continuity)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    for row in &certification.rows[..6] {
        assert!(matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
                | CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
        ));
    }
    assert!(matches!(
        certification.rows[10].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::CubicCubicG2PointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::CubicCubicG2TangentCrossProductPredicate
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::CubicCubicG2CurvatureMagnitudePolynomial
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::CubicCubicG2CurvatureSignPredicate
    );
    for form in [
        &forms.forms[0],
        &forms.forms[1],
        &forms.forms[2],
        &forms.forms[4],
        &forms.forms[5],
    ] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[3]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_arc_cubic_second_order_contact_lowers_exact_curvature_row() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let p1 = sketch.add_point2d("p1", real(5), real(10));
    let p2 = sketch.add_point2d("p2", real(-25), real(20));
    let p3 = sketch.add_point2d("p3", real(-55), real(30));
    let bad_p2 = sketch.add_point2d("bad p2", real(5), real(20));
    let bad_p3 = sketch.add_point2d("bad p3", real(5), real(30));
    let cubic = sketch.add_cubic2("cubic", start, p1, p2, p3);
    let bad_cubic = sketch.add_cubic2("bad cubic", start, p1, bad_p2, bad_p3);
    let t0 = sketch.add_parameter("t0", real(0));
    let valid = sketch_tangency_builders::arc_cubic_second_order_contact2(
        &mut sketch,
        "arc cubic second order",
        arc,
        SketchArcEndpoint::Start,
        cubic,
        t0,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_cubic_second_order_contact2(
        "tangent only",
        arc,
        SketchArcEndpoint::Start,
        bad_cubic,
        t0,
        SketchTangentOrientation::CounterClockwise,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ArcCubicSecondOrderContact
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 12);
    assert_eq!(lowered.rows.len(), 12);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ArcCubicSecondOrderContact)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    for row in &certification.rows[..4] {
        assert!(matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ));
    }
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::ArcCubicSecondOrderEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ArcCubicSecondOrderRadiusPerpendicularPolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ArcCubicSecondOrderOrientationPredicate
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ArcCubicSecondOrderContactPolynomial
    );
    for form in &forms.forms[..4] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[4]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
    assert_eq!(
        forms.forms[5]
            .residual
            .eval_real(context.bindings())
            .unwrap(),
        Real::zero()
    );
}

#[test]
fn sketch_arc_arc_tangent_lowers_exact_radius_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let first_center = sketch.add_point2d("first center", real(0), real(0));
    let shared = sketch.add_point2d("shared", real(5), real(0));
    let first_end = sketch.add_point2d("first end", real(0), real(5));
    let first_radius = sketch.add_distance("first radius", real(5));
    let first =
        sketch.add_arc_of_circle2("first arc", first_center, shared, first_end, first_radius);
    let second_center = sketch.add_point2d("second center", real(10), real(0));
    let second_end = sketch.add_point2d("second end", real(10), real(5));
    let second_radius = sketch.add_distance("second radius", real(5));
    let second = sketch.add_arc_of_circle2(
        "second arc",
        second_center,
        shared,
        second_end,
        second_radius,
    );
    let skew_center = sketch.add_point2d("skew center", real(5), real(5));
    let skew_end = sketch.add_point2d("skew end", real(10), real(5));
    let skew = sketch.add_arc_of_circle2("skew arc", skew_center, shared, skew_end, second_radius);
    let valid = sketch_tangency_builders::arc_arc_tangent2(
        &mut sketch,
        "arc arc tangent",
        first,
        SketchArcEndpoint::Start,
        second,
        SketchArcEndpoint::Start,
        SketchArcTangencyBranch::OppositeRadiusDirection,
    );
    sketch.add_arc_arc_tangent2(
        "wrong branch",
        first,
        SketchArcEndpoint::Start,
        second,
        SketchArcEndpoint::Start,
        SketchArcTangencyBranch::SameRadiusDirection,
    );
    sketch.add_arc_arc_tangent2(
        "skew radius",
        first,
        SketchArcEndpoint::Start,
        skew,
        SketchArcEndpoint::Start,
        SketchArcTangencyBranch::OppositeRadiusDirection,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(valid.strategy, SketchResidualStrategy::ArcArcTangent);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 18);
    assert_eq!(lowered.rows.len(), 18);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ArcArcTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    for row in &certification.rows[..5] {
        assert!(matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ));
    }
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[16].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::ArcArcTangentEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ArcArcTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ArcArcTangentRadiusCrossProductPredicate
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ArcArcTangentRadiusBranchPredicate
    );
    for form in &forms.forms[..5] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[5]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_cubic_cubic_c2_lowers_exact_second_derivative_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(1), real(0));
    let a2 = sketch.add_point2d("a2", real(2), real(0));
    let join = sketch.add_point2d("join", real(3), real(0));
    let b1 = sketch.add_point2d("b1", real(4), real(0));
    let b2 = sketch.add_point2d("b2", real(5), real(0));
    let b3 = sketch.add_point2d("b3", real(6), real(0));
    let bad_b2 = sketch.add_point2d("bad b2", real(5), real(1));
    let first = sketch.add_cubic2("first", a0, a1, a2, join);
    let second = sketch.add_cubic2("second", join, b1, b2, b3);
    let bad_second = sketch.add_cubic2("bad second", join, b1, bad_b2, b3);
    let t_end = sketch.add_parameter("t_end", real(1));
    let t_start = sketch.add_parameter("t_start", real(0));
    let valid = sketch_tangency_builders::cubic_cubic_c2_continuity2(
        &mut sketch,
        "cubic c2",
        first,
        t_end,
        second,
        t_start,
    );
    sketch.add_cubic_cubic_c2_continuity2("bad cubic c2", first, t_end, bad_second, t_start);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::CubicCubicC2Continuity
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);
    let context = context_from_problem(&lowered.problem);

    assert_eq!(lowered.problem.constraints.len(), 12);
    assert_eq!(lowered.rows.len(), 12);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicCubicC2Continuity)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    for row in &certification.rows[..6] {
        assert!(matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ));
    }
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::CubicCubicC2PointPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::CubicCubicC2FirstDerivativePolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::CubicCubicC2SecondDerivativePolynomial
    );
    for form in &forms.forms {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
}

#[test]
fn sketch_equal_angle_constraints_retain_exact_and_proposal_residual_forms() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x = sketch.add_point2d("x", real(5), real(0));
    let diagonal = sketch.add_point2d("diagonal", real(3), real(4));
    let other_origin = sketch.add_point2d("other origin", real(10), real(0));
    let other_x = sketch.add_point2d("other x", real(15), real(0));
    let other_diagonal = sketch.add_point2d("other diagonal", real(13), real(4));
    let line_x = sketch.add_line_segment2("x", origin, x);
    let line_diagonal = sketch.add_line_segment2("diagonal", origin, diagonal);
    let other_line_x = sketch.add_line_segment2("other x", other_origin, other_x);
    let other_line_diagonal =
        sketch.add_line_segment2("other diagonal", other_origin, other_diagonal);
    let angle = sketch_angle_builders::equal_angle_lines2(
        &mut sketch,
        "equal angle",
        line_x,
        line_diagonal,
        other_line_x,
        other_line_diagonal,
    );

    let forms = sketch.residual_forms_for_constraint(angle.handle);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::SquaredCosineAnglePolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::SquaredCosineAngleEquality)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::TrueAngleProposal
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
}

#[test]
fn sketch_oriented_angle_constraints_retain_exact_branch_residual_forms() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x = sketch.add_point2d("x", real(4), real(0));
    let y = sketch.add_point2d("y", real(0), real(4));
    let shifted_origin = sketch.add_point2d("shifted origin", real(10), real(2));
    let shifted_x = sketch.add_point2d("shifted x", real(16), real(2));
    let shifted_y = sketch.add_point2d("shifted y", real(10), real(8));
    let first_a = sketch.add_line_segment2("first a", origin, x);
    let first_b = sketch.add_line_segment2("first b", origin, y);
    let second_a = sketch.add_line_segment2("second a", shifted_origin, shifted_x);
    let second_b = sketch.add_line_segment2("second b", shifted_origin, shifted_y);
    let angle = sketch_angle_builders::equal_oriented_angle_lines2(
        &mut sketch,
        "oriented angle",
        first_a,
        first_b,
        second_a,
        second_b,
    );

    let forms = sketch.residual_forms_for_constraint(angle.handle);
    let context = context_from_problem(&sketch.lower_to_problem().problem);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::OrientedAngleVectorCollinearityPolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::OrientedAngleEquality)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::OrientedAngleSameBranchPredicate
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[1].strategy,
        Some(SketchResidualStrategy::OrientedAngleEquality)
    );
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
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
    );
}

#[test]
fn sketch_tangent_constraints_retain_exact_predicate_residual_forms() {
    let mut sketch = SketchSolveProblem::new();
    let candidate_start = sketch.add_point2d("candidate start", real(0), real(0));
    let candidate_end = sketch.add_point2d("candidate end", real(3), real(4));
    let target_start = sketch.add_point2d("target start", real(2), real(1));
    let target_end = sketch.add_point2d("target end", real(8), real(9));
    let candidate = sketch.add_line_segment2("candidate tangent", candidate_start, candidate_end);
    let target = sketch.add_line_segment2("target tangent", target_start, target_end);
    let tangent = sketch_tangency_builders::tangent_same_direction_lines2(
        &mut sketch,
        "same tangent",
        candidate,
        target,
    );

    let forms = sketch.residual_forms_for_constraint(tangent.handle);

    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert!(forms.diagnostics.is_empty());
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::TangentCrossProductPredicate
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[0].strategy,
        Some(SketchResidualStrategy::TangentSameDirection)
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::TangentDotProductPredicate
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[1].strategy,
        Some(SketchResidualStrategy::TangentSameDirection)
    );

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
            .unwrap()
            .structural_facts()
            .sign,
        Some(hyperreal::RealSign::Positive)
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
    let bad_angle = sketch_angle_builders::equal_angle_lines2(
        &mut sketch,
        "bad angle",
        point,
        point,
        point,
        point,
    );
    let bad_tangent = sketch_tangency_builders::tangent_same_direction_lines2(
        &mut sketch,
        "bad tangent",
        point,
        point,
    );

    assert_eq!(
        sketch.residual_forms_for_constraint(horizontal).status,
        SketchResidualFormsStatus::UnsupportedConstraint
    );

    let bad_forms = sketch.residual_forms_for_constraint(bad.handle);
    assert_eq!(bad_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(bad_forms.forms.is_empty());
    assert!(!bad_forms.diagnostics.is_empty());
    let bad_point_line =
        sketch.add_point_line_distance2("bad point line", point, distance, distance);
    let bad_point_line_forms = sketch.residual_forms_for_constraint(bad_point_line);
    assert_eq!(
        bad_point_line_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_point_line_forms.forms.is_empty());
    assert!(!bad_point_line_forms.diagnostics.is_empty());
    let bad_circle = sketch.add_point_on_circle("bad circle", point, distance);
    let bad_circle_forms = sketch.residual_forms_for_constraint(bad_circle);
    assert_eq!(
        bad_circle_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_circle_forms.forms.is_empty());
    assert!(!bad_circle_forms.diagnostics.is_empty());
    let t = sketch.add_parameter("t", real(0));
    let bad_cubic = sketch.add_point_on_cubic2("bad cubic", point, distance, t);
    let bad_cubic_forms = sketch.residual_forms_for_constraint(bad_cubic);
    assert_eq!(
        bad_cubic_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_cubic_forms.forms.is_empty());
    assert!(!bad_cubic_forms.diagnostics.is_empty());
    let bad_cubic_tangent = sketch.add_cubic_line_tangent2(
        "bad cubic tangent",
        distance,
        t,
        distance,
        SketchLineEndpoint::Start,
    );
    let bad_cubic_tangent_forms = sketch.residual_forms_for_constraint(bad_cubic_tangent);
    assert_eq!(
        bad_cubic_tangent_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_cubic_tangent_forms.forms.is_empty());
    assert!(!bad_cubic_tangent_forms.diagnostics.is_empty());
    let bad_cubic_cubic_tangent =
        sketch.add_cubic_cubic_tangent2("bad cubic cubic", distance, t, distance, t);
    let bad_cubic_cubic_tangent_forms =
        sketch.residual_forms_for_constraint(bad_cubic_cubic_tangent);
    assert_eq!(
        bad_cubic_cubic_tangent_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_cubic_cubic_tangent_forms.forms.is_empty());
    assert!(!bad_cubic_cubic_tangent_forms.diagnostics.is_empty());
    let bad_cubic_g2 = sketch.add_cubic_cubic_g2_continuity2("bad g2", distance, t, distance, t);
    let bad_cubic_g2_forms = sketch.residual_forms_for_constraint(bad_cubic_g2);
    assert_eq!(
        bad_cubic_g2_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_cubic_g2_forms.forms.is_empty());
    assert!(!bad_cubic_g2_forms.diagnostics.is_empty());
    let bad_arc_cubic_tangent = sketch.add_arc_cubic_tangent2(
        "bad arc cubic",
        distance,
        SketchArcEndpoint::Start,
        distance,
        t,
        SketchTangentOrientation::CounterClockwise,
    );
    let bad_arc_cubic_tangent_forms = sketch.residual_forms_for_constraint(bad_arc_cubic_tangent);
    assert_eq!(
        bad_arc_cubic_tangent_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_arc_cubic_tangent_forms.forms.is_empty());
    assert!(!bad_arc_cubic_tangent_forms.diagnostics.is_empty());
    let bad_arc_arc_tangent = sketch.add_arc_arc_tangent2(
        "bad arc arc",
        distance,
        SketchArcEndpoint::Start,
        distance,
        SketchArcEndpoint::Start,
        SketchArcTangencyBranch::SameRadiusDirection,
    );
    let bad_arc_arc_tangent_forms = sketch.residual_forms_for_constraint(bad_arc_arc_tangent);
    assert_eq!(
        bad_arc_arc_tangent_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_arc_arc_tangent_forms.forms.is_empty());
    assert!(!bad_arc_arc_tangent_forms.diagnostics.is_empty());
    let bad_arc_cubic_second = sketch.add_arc_cubic_second_order_contact2(
        "bad arc cubic second",
        distance,
        SketchArcEndpoint::Start,
        distance,
        t,
        SketchTangentOrientation::CounterClockwise,
    );
    let bad_arc_cubic_second_forms = sketch.residual_forms_for_constraint(bad_arc_cubic_second);
    assert_eq!(
        bad_arc_cubic_second_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_arc_cubic_second_forms.forms.is_empty());
    assert!(!bad_arc_cubic_second_forms.diagnostics.is_empty());
    let bad_cubic_c2 = sketch.add_cubic_cubic_c2_continuity2("bad c2", distance, t, distance, t);
    let bad_cubic_c2_forms = sketch.residual_forms_for_constraint(bad_cubic_c2);
    assert_eq!(
        bad_cubic_c2_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_cubic_c2_forms.forms.is_empty());
    assert!(!bad_cubic_c2_forms.diagnostics.is_empty());
    let bad_angle_forms = sketch.residual_forms_for_constraint(bad_angle.handle);
    assert_eq!(
        bad_angle_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_angle_forms.forms.is_empty());
    assert!(!bad_angle_forms.diagnostics.is_empty());
    let bad_tangent_forms = sketch.residual_forms_for_constraint(bad_tangent.handle);
    assert_eq!(
        bad_tangent_forms.status,
        SketchResidualFormsStatus::InvalidInputs
    );
    assert!(bad_tangent_forms.forms.is_empty());
    assert!(!bad_tangent_forms.diagnostics.is_empty());
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

    assert_eq!(fixtures.len(), 6);
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
            ..SketchRoundTripMetadata::default()
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
fn sketch_unit_tolerance_audit_keeps_tolerances_explicit_and_report_bearing() {
    let mut sketch = SketchSolveProblem::new();
    let parameter = sketch.add_parameter("width", real(10));
    let point = sketch.add_point2d("p", real(0), real(0));
    let distance = sketch.add_distance("d", real(5));
    let constraint = sketch.add_point_point_distance("dimension", point, point, distance);

    assert!(sketch.set_parameter_metadata(
        parameter,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            declared_tolerance: Some(real(1)),
            display_label: Some("width".to_owned()),
            ..SketchRoundTripMetadata::default()
        },
    ));
    assert!(sketch.set_entity_metadata(
        point,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            ..SketchRoundTripMetadata::default()
        },
    ));
    assert!(sketch.set_constraint_metadata(
        constraint,
        SketchRoundTripMetadata {
            declared_tolerance: Some(real(0)),
            display_label: Some("missing unit tolerance".to_owned()),
            ..SketchRoundTripMetadata::default()
        },
    ));
    assert!(sketch.set_parameter_metadata(
        sketch.parameters()[1].handle,
        SketchRoundTripMetadata {
            source_unit: Some("mm".to_owned()),
            declared_tolerance: Some(real(-1)),
            ..SketchRoundTripMetadata::default()
        },
    ));

    let report = audit_sketch_unit_tolerances(&sketch);

    assert_eq!(
        report.object_count,
        sketch.parameters().len() + sketch.entities().len() + sketch.constraints().len()
    );
    assert_eq!(report.certified_declared_tolerances, 1);
    assert_eq!(report.invalid_declarations, 2);
    assert!(report.has_invalid_declaration());
    assert!(report.rows.iter().any(|row| {
        row.parameter == Some(parameter)
            && row.source_unit.as_deref() == Some("mm")
            && row.declared_tolerance == Some(real(1))
            && row.status == SketchUnitToleranceStatus::CertifiedDeclaredTolerance
    }));
    assert!(report.rows.iter().any(|row| {
        row.entity == Some(point) && row.status == SketchUnitToleranceStatus::UnitOnly
    }));
    assert!(report.rows.iter().any(|row| {
        row.constraint == Some(constraint)
            && row.status == SketchUnitToleranceStatus::ToleranceWithoutUnit
    }));
    assert!(
        report.rows.iter().any(|row| {
            row.status == SketchUnitToleranceStatus::CertifiedInvalidNegativeTolerance
        })
    );
}

#[test]
fn sketch_range_and_soft_objective_builders_lower_to_exact_row_kinds() {
    let mut sketch = SketchSolveProblem::new();
    let parameter = sketch.add_parameter("t", real(3));
    let lower_parameter = sketch.add_parameter("lower", real(1));
    let range = sketch_range_builders::parameter_range(
        &mut sketch,
        "bounded t",
        parameter,
        Some(real(1)),
        Some(real(5)),
    );
    let ordering = sketch_range_builders::parameter_ordering(
        &mut sketch,
        "nondecreasing",
        lower_parameter,
        parameter,
    );
    let objective = sketch_objective_builders::stay_near_parameter(
        &mut sketch,
        "stay near",
        parameter,
        real(3),
        real(7),
    );

    assert_eq!(range.family, hypersolve::SketchConstraintFamily::Range);
    assert_eq!(ordering.family, hypersolve::SketchConstraintFamily::Range);
    assert_eq!(ordering.strategy, SketchResidualStrategy::ParameterOrdering);
    assert_eq!(
        objective.family,
        hypersolve::SketchConstraintFamily::Objective
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.problem.constraints[0].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        lowered.problem.constraints[1].kind,
        ConstraintKind::LessOrEqual
    );
    assert_eq!(
        lowered.problem.constraints[2].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(lowered.problem.constraints[3].kind, ConstraintKind::Soft);
    assert_eq!(lowered.problem.constraints[3].weight, real(7));
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::ParameterRange)
    );
    assert_eq!(
        lowered.rows[2].strategy,
        Some(SketchResidualStrategy::ParameterOrdering)
    );
    assert_eq!(
        lowered.rows[3].strategy,
        Some(SketchResidualStrategy::SoftObjective)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(certification.all_satisfied());
}

#[test]
fn sketch_parameter_ordering_reports_violations_and_stale_parameters_explicitly() {
    let mut sketch = SketchSolveProblem::new();
    let high = sketch.add_parameter("high", real(5));
    let low = sketch.add_parameter("low", real(1));
    sketch.add_parameter_ordering("violated", high, low);
    sketch.add_parameter_ordering("stale", high, hypersolve::SketchParameterHandle(999));

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 1);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::ParameterOrdering)
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::MissingParameter(hypersolve::SketchParameterHandle(999))
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_parameter_margins_validate_exact_inputs_and_replay_candidates() {
    let mut sketch = SketchSolveProblem::new();
    let low = sketch.add_parameter("low", real(1));
    let high = sketch.add_parameter("high", real(5));
    let valid = sketch_range_builders::parameter_margin(
        &mut sketch,
        "clearance margin",
        low,
        high,
        real(3),
    );
    sketch.add_parameter_margin("violated margin", low, high, real(5));
    sketch.add_parameter_margin("negative margin", low, high, real(-1));
    sketch.add_parameter_margin(
        "stale margin",
        low,
        hypersolve::SketchParameterHandle(999),
        real(1),
    );

    assert_eq!(valid.strategy, SketchResidualStrategy::ParameterMargin);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::ParameterMargin)
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingParameter(hypersolve::SketchParameterHandle(999))
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
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
fn sketch_line_orientation_relations_lower_to_exact_dot_and_cross_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x_axis = sketch.add_point2d("x axis", real(3), real(0));
    let x_parallel_start = sketch.add_point2d("parallel start", real(0), real(2));
    let x_parallel_end = sketch.add_point2d("parallel end", real(5), real(2));
    let y_axis = sketch.add_point2d("y axis", real(0), real(4));
    let diagonal = sketch.add_point2d("diagonal", real(1), real(1));
    let horizontal = sketch.add_line_segment2("horizontal", origin, x_axis);
    let parallel = sketch.add_line_segment2("parallel", x_parallel_start, x_parallel_end);
    let vertical = sketch.add_line_segment2("vertical", origin, y_axis);
    let slanted = sketch.add_line_segment2("slanted", origin, diagonal);
    let parallel_report = sketch_orientation_builders::parallel_lines2(
        &mut sketch,
        "parallel lines",
        horizontal,
        parallel,
    );
    let perpendicular_report = sketch_orientation_builders::perpendicular_lines2(
        &mut sketch,
        "perpendicular lines",
        horizontal,
        vertical,
    );
    sketch.add_parallel_lines2("violated parallel", horizontal, slanted);

    assert_eq!(
        parallel_report.family,
        hypersolve::SketchConstraintFamily::Orientation
    );
    assert_eq!(
        parallel_report.strategy,
        SketchResidualStrategy::DirectionCrossProduct
    );
    assert_eq!(
        perpendicular_report.strategy,
        SketchResidualStrategy::DirectionDotProduct
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::DirectionCrossProduct)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::DirectionDotProduct)
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_line_orientation_relations_report_stale_and_wrong_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    sketch.add_parallel_lines2("missing line", line, SketchEntityHandle(999));
    sketch.add_perpendicular_lines2("wrong family", line, p);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
}

#[test]
fn sketch_same_direction_relations_lower_to_parallel_and_orientation_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(3), real(0));
    let b0 = sketch.add_point2d("b0", real(1), real(2));
    let b1 = sketch.add_point2d("b1", real(5), real(2));
    let c0 = sketch.add_point2d("c0", real(5), real(4));
    let c1 = sketch.add_point2d("c1", real(1), real(4));
    let same_a = sketch.add_line_segment2("same a", a0, a1);
    let same_b = sketch.add_line_segment2("same b", b0, b1);
    let reversed = sketch.add_line_segment2("reversed", c0, c1);
    let same_report = sketch_orientation_builders::same_direction_lines2(
        &mut sketch,
        "same direction",
        same_a,
        same_b,
    );
    sketch.add_same_direction_lines2("opposite direction", same_a, reversed);

    assert_eq!(
        same_report.strategy,
        SketchResidualStrategy::DirectionSameOrientation
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::DirectionSameOrientation)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation {
            sign: hyperreal::RealSign::Positive,
            ..
        }
    ));
}

#[test]
fn sketch_same_direction_relations_report_stale_and_wrong_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    sketch.add_same_direction_lines2("missing line", line, SketchEntityHandle(999));
    sketch.add_same_direction_lines2("wrong family", line, p);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
}

#[test]
fn sketch_tangent_same_direction_relations_lower_to_g1_predicate_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("candidate start", real(0), real(0));
    let a1 = sketch.add_point2d("candidate end", real(3), real(4));
    let b0 = sketch.add_point2d("target start", real(2), real(1));
    let b1 = sketch.add_point2d("target end", real(8), real(9));
    let c0 = sketch.add_point2d("reverse start", real(8), real(9));
    let c1 = sketch.add_point2d("reverse end", real(2), real(1));
    let candidate = sketch.add_line_segment2("candidate tangent", a0, a1);
    let target = sketch.add_line_segment2("target tangent", b0, b1);
    let reversed = sketch.add_line_segment2("reversed tangent", c0, c1);
    let report = sketch_tangency_builders::tangent_same_direction_lines2(
        &mut sketch,
        "same tangent",
        candidate,
        target,
    );
    sketch.add_tangent_same_direction_lines2("opposite tangent", candidate, reversed);

    assert_eq!(report.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(
        report.strategy,
        SketchResidualStrategy::TangentSameDirection
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::TangentSameDirection)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_tangent_same_direction_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let point3 = sketch.add_point3d("point3", real(1), real(0), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    let non_2d = sketch.add_line_segment2("non 2d", point3, q);
    sketch.add_tangent_same_direction_lines2("missing tangent", line, SketchEntityHandle(999));
    sketch.add_tangent_same_direction_lines2("wrong tangent family", line, p);
    sketch.add_tangent_same_direction_lines2("non 2d tangent", line, non_2d);
    sketch.add_tangent_same_direction_lines2("valid control", line, line);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 5);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: non_2d,
            expected: "2D line segment"
        }
    );
    assert_eq!(lowered.rows[3].status, SketchGeneratedRowStatus::Generated);
    assert_eq!(lowered.rows[4].status, SketchGeneratedRowStatus::Generated);
}

#[test]
fn sketch_arc_line_tangent_relations_lower_endpoint_radius_and_orientation_rows() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let tangent_end = sketch.add_point2d("tangent end", real(5), real(3));
    let tangent = sketch.add_line_segment2("tangent", start, tangent_end);
    let radial_end = sketch.add_point2d("radial end", real(8), real(0));
    let radial = sketch.add_line_segment2("radial", start, radial_end);
    let clockwise_end = sketch.add_point2d("clockwise end", real(5), real(-3));
    let clockwise = sketch.add_line_segment2("clockwise", start, clockwise_end);
    let valid = sketch_tangency_builders::arc_line_tangent2(
        &mut sketch,
        "arc line tangent",
        arc,
        SketchArcEndpoint::Start,
        tangent,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "bad radial tangent",
        arc,
        SketchArcEndpoint::Start,
        radial,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "bad orientation tangent",
        arc,
        SketchArcEndpoint::Start,
        clockwise,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "clockwise tangent",
        arc,
        SketchArcEndpoint::Start,
        clockwise,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::Clockwise,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Tangency);
    assert_eq!(valid.strategy, SketchResidualStrategy::ArcLineTangent);

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 20);
    assert_eq!(lowered.rows.len(), 20);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ArcLineTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[8].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[14].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[19].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::ArcLineTangentEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ArcLineTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ArcLineTangentRadiusPerpendicularPolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ArcLineTangentOrientationPredicate
    );
    assert!(forms.forms.iter().all(|form| {
        form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::ArcLineTangent)
    }));
}

#[test]
fn sketch_projected_arc_line_tangent_replays_workplane_endpoint_and_orientation_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let line_start = sketch.add_point3d("line start", real(6), real(2), real(-7));
    let line_end = sketch.add_point3d("line end", real(6), real(8), real(11));
    let line = sketch.add_line_segment3("projected line", line_start, line_end);
    let valid = sketch_tangency_builders::projected_arc_line_tangent3(
        &mut sketch,
        "projected arc line tangent",
        workplane,
        arc,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_projected_arc_line_tangent3(
        "wrong projected tangent orientation",
        workplane,
        arc,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::Clockwise,
    );
    let wrong = sketch.add_projected_arc_line_tangent3(
        "wrong projected tangent line kind",
        workplane,
        arc,
        SketchArcEndpoint::Start,
        start,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedArcLineTangent
    );
    assert_eq!(lowered.problem.constraints.len(), 12);
    assert!(lowered.rows[..12].iter().all(|row| row.strategy
        == Some(SketchResidualStrategy::ProjectedArcLineTangent)
        || row.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion)));
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: start,
            expected: "3D line segment",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.rows[..6].iter().all(|row| {
        matches!(
            row.status,
            CertifiedCandidateStatus::CertifiedZero { .. }
                | CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
        )
    }));
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedArcLineTangentEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedArcLineTangentEndpointIncidencePolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedArcLineTangentRadiusPerpendicularPolynomial
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ProjectedArcLineTangentOrientationPredicate
    );
    assert!(
        forms
            .forms
            .iter()
            .all(|form| form.role == SketchResidualFormRole::ExactProof)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_arc_line_tangent_supports_end_endpoint_and_line_end_orientation() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let tangent_start = sketch.add_point2d("tangent start", real(-3), real(5));
    let tangent = sketch.add_line_segment2("tangent", tangent_start, end);
    let handle = sketch.add_arc_line_tangent2(
        "arc end to line end tangent",
        arc,
        SketchArcEndpoint::End,
        tangent,
        SketchLineEndpoint::End,
        SketchTangentOrientation::CounterClockwise,
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(handle);

    assert_eq!(lowered.rows.len(), 5);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ArcLineTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(certification.all_satisfied());
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 5);
}

#[test]
fn sketch_arc_line_tangent_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let line_end = sketch.add_point2d("line end", real(5), real(3));
    let line = sketch.add_line_segment2("line", start, line_end);
    let point3 = sketch.add_point3d("point3", real(5), real(0), real(0));
    let non_2d_line = sketch.add_line_segment2("non 2d line", point3, line_end);
    sketch.add_arc_line_tangent2(
        "missing arc",
        SketchEntityHandle(999),
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "wrong arc family",
        start,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "wrong line family",
        arc,
        SketchArcEndpoint::Start,
        start,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "non 2d line",
        arc,
        SketchArcEndpoint::Start,
        non_2d_line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    sketch.add_arc_line_tangent2(
        "valid control",
        arc,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 5);
    assert_eq!(lowered.rows.len(), 9);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: start,
            expected: "2D circular arc"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: start,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: non_2d_line,
            expected: "2D line segment"
        }
    );
    assert!(lowered.rows[4..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ArcLineTangent)
    }));
}

#[test]
fn sketch_arc_line_tangent_rejects_inconsistent_arc_radius() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(6), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("inconsistent arc", center, start, end, radius);
    let line_end = sketch.add_point2d("line end", real(6), real(3));
    let line = sketch.add_line_segment2("line", start, line_end);
    sketch.add_arc_line_tangent2(
        "inconsistent arc tangent",
        arc,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.rows.len(), 5);
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
}

#[test]
fn sketch_equal_angle_relations_lower_to_exact_squared_cosine_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x = sketch.add_point2d("x", real(5), real(0));
    let diag = sketch.add_point2d("diag", real(3), real(4));
    let other_origin = sketch.add_point2d("other origin", real(10), real(2));
    let other_x = sketch.add_point2d("other x", real(20), real(2));
    let other_diag = sketch.add_point2d("other diag", real(16), real(10));
    let y = sketch.add_point2d("y", real(0), real(5));
    let line_x = sketch.add_line_segment2("x axis", origin, x);
    let line_diag = sketch.add_line_segment2("diag", origin, diag);
    let line_other_x = sketch.add_line_segment2("other x", other_origin, other_x);
    let line_other_diag = sketch.add_line_segment2("other diag", other_origin, other_diag);
    let line_y = sketch.add_line_segment2("y", origin, y);
    let equal_angle = sketch_angle_builders::equal_angle_lines2(
        &mut sketch,
        "equal angle",
        line_x,
        line_diag,
        line_other_x,
        line_other_diag,
    );
    sketch.add_equal_angle_lines2("violated angle", line_x, line_diag, line_x, line_y);

    assert_eq!(
        equal_angle.family,
        hypersolve::SketchConstraintFamily::Angle
    );
    assert_eq!(
        equal_angle.strategy,
        SketchResidualStrategy::SquaredCosineAngleEquality
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 2);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::SquaredCosineAngleEquality)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_oriented_angle_relations_lower_to_exact_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x = sketch.add_point2d("x", real(4), real(0));
    let y = sketch.add_point2d("y", real(0), real(4));
    let shifted_origin = sketch.add_point2d("shifted origin", real(10), real(2));
    let shifted_x = sketch.add_point2d("shifted x", real(16), real(2));
    let shifted_y = sketch.add_point2d("shifted y", real(10), real(8));
    let shifted_neg_y = sketch.add_point2d("shifted neg y", real(10), real(-4));
    let diagonal = sketch.add_point2d("diagonal", real(4), real(4));
    let first_a = sketch.add_line_segment2("first a", origin, x);
    let first_b = sketch.add_line_segment2("first b", origin, y);
    let second_a = sketch.add_line_segment2("second a", shifted_origin, shifted_x);
    let second_b = sketch.add_line_segment2("second b", shifted_origin, shifted_y);
    let reversed_b = sketch.add_line_segment2("reversed b", shifted_origin, shifted_neg_y);
    let diagonal_b = sketch.add_line_segment2("diagonal b", origin, diagonal);
    let valid = sketch_angle_builders::equal_oriented_angle_lines2(
        &mut sketch,
        "oriented angle",
        first_a,
        first_b,
        second_a,
        second_b,
    );
    sketch.add_equal_oriented_angle_lines2(
        "unsigned only violation",
        first_a,
        first_b,
        second_a,
        reversed_b,
    );
    sketch.add_equal_oriented_angle_lines2(
        "angle magnitude violation",
        first_a,
        first_b,
        first_a,
        diagonal_b,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Angle);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::OrientedAngleEquality
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(lowered.rows.len(), 6);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::OrientedAngleEquality)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert!(forms.forms.iter().all(|form| {
        form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::OrientedAngleEquality)
    }));
}

#[test]
fn sketch_equal_angle_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let r = sketch.add_point2d("r", real(0), real(1));
    let point3 = sketch.add_point3d("point3", real(0), real(0), real(1));
    let line = sketch.add_line_segment2("line", p, q);
    let other = sketch.add_line_segment2("other", p, r);
    let not_2d = sketch.add_line_segment2("not 2d", p, point3);
    sketch.add_equal_angle_lines2("missing", line, other, line, SketchEntityHandle(999));
    sketch.add_equal_angle_lines2("wrong family", line, other, line, p);
    sketch.add_equal_angle_lines2("not 2d", line, other, line, not_2d);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: not_2d,
            expected: "2D line segment"
        }
    );
}

#[test]
fn sketch_oriented_angle_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let r = sketch.add_point2d("r", real(0), real(1));
    let point3 = sketch.add_point3d("point3", real(0), real(0), real(1));
    let line = sketch.add_line_segment2("line", p, q);
    let other = sketch.add_line_segment2("other", p, r);
    let not_2d = sketch.add_line_segment2("not 2d", p, point3);
    sketch.add_equal_oriented_angle_lines2("missing", line, other, line, SketchEntityHandle(999));
    sketch.add_equal_oriented_angle_lines2("wrong family", line, other, line, p);
    sketch.add_equal_oriented_angle_lines2("not 2d", line, other, line, not_2d);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: not_2d,
            expected: "2D line segment"
        }
    );
}

#[test]
fn sketch_midpoint_relations_lower_to_exact_linear_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(4), real(6));
    let midpoint = sketch.add_point2d("midpoint", real(2), real(3));
    let wrong = sketch.add_point2d("wrong", real(2), real(4));
    let midpoint_report =
        sketch_symmetry_builders::at_midpoint2(&mut sketch, "midpoint", midpoint, a, b);
    sketch.add_at_midpoint2("violated midpoint", wrong, a, b);

    assert_eq!(
        midpoint_report.family,
        hypersolve::SketchConstraintFamily::Symmetry
    );
    assert_eq!(
        midpoint_report.strategy,
        SketchResidualStrategy::MidpointCoordinateEquality
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::MidpointCoordinateEquality)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_midpoint_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(2), real(2));
    let point3 = sketch.add_point3d("point3", real(1), real(1), real(1));
    let distance = sketch.add_distance("distance", real(1));
    sketch.add_at_midpoint2("missing endpoint", p, q, SketchEntityHandle(999));
    sketch.add_at_midpoint2("wrong family", p, q, distance);
    sketch.add_at_midpoint2("not 2d", p, q, point3);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: distance,
            expected: "point"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point3,
            expected: "2D point"
        }
    );
}

#[test]
fn sketch_axis_symmetry_relations_lower_to_exact_linear_rows() {
    let mut sketch = SketchSolveProblem::new();
    let top = sketch.add_point2d("top", real(2), real(7));
    let bottom = sketch.add_point2d("bottom", real(2), real(1));
    let left = sketch.add_point2d("left", real(-4), real(3));
    let right = sketch.add_point2d("right", real(6), real(3));
    let wrong = sketch.add_point2d("wrong", real(3), real(1));
    let horizontal = sketch_symmetry_builders::symmetric_horizontal2(
        &mut sketch,
        "horizontal symmetry",
        top,
        bottom,
        real(4),
    );
    let vertical = sketch_symmetry_builders::symmetric_vertical2(
        &mut sketch,
        "vertical symmetry",
        left,
        right,
        real(1),
    );
    sketch.add_symmetric_horizontal2("violated symmetry", top, wrong, real(4));

    assert_eq!(
        horizontal.family,
        hypersolve::SketchConstraintFamily::Symmetry
    );
    assert_eq!(
        horizontal.strategy,
        SketchResidualStrategy::AxisSymmetryCoordinateEquality
    );
    assert_eq!(
        vertical.strategy,
        SketchResidualStrategy::AxisSymmetryCoordinateEquality
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(lowered.rows.len(), 6);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::AxisSymmetryCoordinateEquality)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
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
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_axis_symmetry_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(1));
    let point3 = sketch.add_point3d("point3", real(1), real(1), real(1));
    let distance = sketch.add_distance("distance", real(1));
    sketch.add_symmetric_horizontal2("missing", p, SketchEntityHandle(999), real(0));
    sketch.add_symmetric_vertical2("wrong family", p, distance, real(0));
    sketch.add_symmetric_horizontal2("not 2d", p, point3, real(0));
    sketch.add_symmetric_vertical2("valid control", p, q, real(0));

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 5);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: distance,
            expected: "point"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point3,
            expected: "2D point"
        }
    );
    assert_eq!(lowered.rows[3].status, SketchGeneratedRowStatus::Generated);
    assert_eq!(lowered.rows[4].status, SketchGeneratedRowStatus::Generated);
}

#[test]
fn sketch_line_symmetry_relations_lower_to_exact_polynomial_rows() {
    let mut sketch = SketchSolveProblem::new();
    let axis_start = sketch.add_point2d("axis start", real(0), real(0));
    let axis_end = sketch.add_point2d("axis end", real(5), real(0));
    let axis = sketch.add_line_segment2("axis", axis_start, axis_end);
    let top = sketch.add_point2d("top", real(2), real(3));
    let bottom = sketch.add_point2d("bottom", real(2), real(-3));
    let wrong = sketch.add_point2d("wrong", real(3), real(-3));
    let valid =
        sketch_symmetry_builders::symmetric_line2(&mut sketch, "line symmetry", top, bottom, axis);
    sketch.add_symmetric_line2("violated line symmetry", top, wrong, axis);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Symmetry);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::LineSymmetryPolynomial
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::LineSymmetryPolynomial)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_line_symmetry_relations_report_stale_wrong_and_non_2d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(1));
    let axis_start = sketch.add_point2d("axis start", real(0), real(0));
    let axis_end = sketch.add_point2d("axis end", real(1), real(0));
    let axis = sketch.add_line_segment2("axis", axis_start, axis_end);
    let point3 = sketch.add_point3d("point3", real(1), real(1), real(1));
    let non_2d_axis = sketch.add_line_segment2("non 2d axis", point3, axis_end);
    sketch.add_symmetric_line2("missing axis", p, q, SketchEntityHandle(999));
    sketch.add_symmetric_line2("wrong axis family", p, q, p);
    sketch.add_symmetric_line2("non 2d axis", p, q, non_2d_axis);
    sketch.add_symmetric_line2("valid control", p, q, axis);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 5);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: non_2d_axis,
            expected: "2D line segment"
        }
    );
    assert_eq!(lowered.rows[3].status, SketchGeneratedRowStatus::Generated);
    assert_eq!(lowered.rows[4].status, SketchGeneratedRowStatus::Generated);
}

#[test]
fn sketch_projected_line_symmetry_replays_workplane_midpoint_and_perpendicular_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(10), real(-2), real(7));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let axis_start = sketch.add_point3d("axis start", real(11), real(0), real(-3));
    let axis_end = sketch.add_point3d("axis end", real(16), real(0), real(13));
    let axis = sketch.add_line_segment3("projected axis", axis_start, axis_end);
    let top = sketch.add_point3d("top", real(14), real(5), real(19));
    let bottom = sketch.add_point3d("bottom", real(14), real(-5), real(-11));
    let wrong = sketch.add_point3d("wrong", real(15), real(-5), real(-11));
    let point2 = sketch.add_point2d("point2", real(0), real(0));
    let valid = sketch_symmetry_builders::projected_symmetric_line3(
        &mut sketch,
        "projected line symmetry",
        workplane,
        top,
        bottom,
        axis,
    );
    sketch.add_projected_symmetric_line3(
        "violated projected line symmetry",
        workplane,
        top,
        wrong,
        axis,
    );
    let wrong_handle = sketch.add_projected_symmetric_line3(
        "wrong projected line axis kind",
        workplane,
        top,
        bottom,
        point2,
    );

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Symmetry);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedLineSymmetryPolynomial
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    let forms = sketch.residual_forms_for_constraint(valid.handle);

    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(lowered.rows.len(), 7);
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point2,
            expected: "3D line segment",
        }
    );
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..3].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedLineSymmetryPolynomial)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(
        certification.rows[..3]
            .iter()
            .all(|row| { matches!(row.status, CertifiedCandidateStatus::CertifiedZero { .. }) })
    );
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedLineSymmetryMidpointOnAxisPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedLineSymmetryPerpendicularOffsetPolynomial
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong_handle);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_workplane_symmetry_relations_lower_to_exact_polynomial_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("xy workplane", origin, normal);
    let top = sketch.add_point3d("top", real(2), real(3), real(5));
    let bottom = sketch.add_point3d("bottom", real(2), real(3), real(-5));
    let wrong_midpoint = sketch.add_point3d("wrong midpoint", real(2), real(3), real(-4));
    let wrong_normal = sketch.add_point3d("wrong normal", real(3), real(3), real(-5));
    let valid = sketch_symmetry_builders::symmetric_workplane3(
        &mut sketch,
        "workplane symmetry",
        top,
        bottom,
        workplane,
    );
    sketch.add_symmetric_workplane3("violated midpoint", top, wrong_midpoint, workplane);
    sketch.add_symmetric_workplane3("violated normal", top, wrong_normal, workplane);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Symmetry);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::WorkplaneSymmetryPolynomial
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 15);
    assert_eq!(lowered.rows.len(), 15);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..5].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[6].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[13].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::WorkplaneSymmetryMidpointPlanePolynomial
    );
    assert!(forms.forms[2..].iter().all(|form| {
        form.kind == SketchResidualFormKind::WorkplaneSymmetryNormalOffsetPolynomial
            && form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial)
    }));
}

#[test]
fn sketch_workplane_symmetry_relations_report_stale_wrong_and_non_3d_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("xy workplane", origin, normal);
    let a = sketch.add_point3d("a", real(0), real(0), real(1));
    let b = sketch.add_point3d("b", real(0), real(0), real(-1));
    let point2 = sketch.add_point2d("point2", real(0), real(0));
    sketch.add_symmetric_workplane3("missing workplane", a, b, SketchEntityHandle(999));
    sketch.add_symmetric_workplane3("wrong workplane family", a, b, a);
    sketch.add_symmetric_workplane3("non 3d point", point2, b, workplane);
    sketch.add_symmetric_workplane3("valid control", a, b, workplane);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 5);
    assert_eq!(lowered.rows.len(), 8);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: a,
            expected: "workplane"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point2,
            expected: "3D point"
        }
    );
    assert!(lowered.rows[3..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && matches!(
                row.strategy,
                Some(
                    SketchResidualStrategy::WorkplaneUnitQuaternion
                        | SketchResidualStrategy::WorkplaneSymmetryPolynomial
                )
            )
    }));
}

#[test]
fn sketch_workplane_symmetry_keeps_nonunit_frame_as_proof_row() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let nonunit = sketch.add_normal3d("nonunit", real(2), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("bad workplane", origin, nonunit);
    let a = sketch.add_point3d("a", real(0), real(0), real(3));
    let b = sketch.add_point3d("b", real(0), real(0), real(-3));
    sketch.add_symmetric_workplane3("nonunit symmetry", a, b, workplane);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_point_distance_ranges_lower_to_exact_squared_inequalities() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(3), real(4));
    let range = sketch_distance_builders::point_point_distance_range(
        &mut sketch,
        "clearance window",
        a,
        b,
        Some(real(4)),
        Some(real(6)),
    );

    assert_eq!(range.family, hypersolve::SketchConstraintFamily::Distance);
    assert_eq!(
        range.strategy,
        SketchResidualStrategy::BoundedSquaredDistance
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.problem.constraints[0].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        lowered.problem.constraints[1].kind,
        ConstraintKind::LessOrEqual
    );
    assert!(lowered.rows.iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::BoundedSquaredDistance)
    }));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert!(certification.all_satisfied());
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
}

#[test]
fn sketch_point_distance_ranges_report_invalid_empty_and_stale_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(1), real(0));
    sketch.add_point_point_distance_range("negative lower", a, b, Some(real(-1)), None);
    sketch.add_point_point_distance_range("inverted", a, b, Some(real(6)), Some(real(4)));
    sketch.add_point_point_distance_range("empty", a, b, None, None);
    sketch.add_point_point_distance_range(
        "stale",
        a,
        SketchEntityHandle(999),
        Some(real(0)),
        Some(real(2)),
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
}

#[test]
fn sketch_equal_length_and_radius_relations_lower_to_exact_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(3), real(4));
    let b0 = sketch.add_point2d("b0", real(10), real(1));
    let b1 = sketch.add_point2d("b1", real(13), real(5));
    let c1 = sketch.add_point2d("c1", real(12), real(1));
    let line_a = sketch.add_line_segment2("line a", a0, a1);
    let line_b = sketch.add_line_segment2("line b", b0, b1);
    let short = sketch.add_line_segment2("short", b0, c1);
    let radius = sketch.add_distance("radius", real(5));
    let other_radius = sketch.add_distance("other radius", real(5));
    let wrong_radius = sketch.add_distance("wrong radius", real(7));
    let circle = sketch.add_circle2("circle", a0, radius);
    let same_circle = sketch.add_circle2("same circle", b0, other_radius);
    let wrong_circle = sketch.add_circle2("wrong circle", b0, wrong_radius);

    let length_report =
        sketch_distance_builders::equal_length_lines2(&mut sketch, "equal length", line_a, line_b);
    let radius_report =
        sketch_distance_builders::equal_radius2(&mut sketch, "equal radius", circle, same_circle);
    sketch.add_equal_length_lines2("violated length", line_a, short);
    sketch.add_equal_radius2("violated radius", circle, wrong_circle);

    assert_eq!(
        length_report.family,
        hypersolve::SketchConstraintFamily::Distance
    );
    assert_eq!(
        length_report.strategy,
        SketchResidualStrategy::SquaredLineLengthEquality
    );
    assert_eq!(
        radius_report.strategy,
        SketchResidualStrategy::RadiusEquality
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::SquaredLineLengthEquality)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::RadiusEquality)
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_equal_length_and_radius_relations_report_bad_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    let radius = sketch.add_distance("radius", real(1));
    let circle = sketch.add_circle2("circle", p, radius);
    let bad_circle = sketch.add_circle2("bad circle", p, line);
    sketch.add_equal_length_lines2("missing line", line, SketchEntityHandle(999));
    sketch.add_equal_length_lines2("wrong length input", line, p);
    sketch.add_equal_radius2("missing radius object", circle, SketchEntityHandle(999));
    sketch.add_equal_radius2("wrong radius object", circle, line);
    sketch.add_equal_radius2("wrong radius carrier", circle, bad_circle);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 5);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "circle or circular arc"
        }
    );
    assert_eq!(
        lowered.rows[4].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "distance"
        }
    );
}

#[test]
fn sketch_length_ratio_and_point_line_distance_lower_to_exact_rows() {
    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", real(0), real(0));
    let a1 = sketch.add_point2d("a1", real(6), real(8));
    let b0 = sketch.add_point2d("b0", real(10), real(0));
    let b1 = sketch.add_point2d("b1", real(13), real(4));
    let short = sketch.add_line_segment2("short", b0, b1);
    let long = sketch.add_line_segment2("long", a0, a1);
    let near = sketch.add_point2d("near", real(2), real(3));
    let wrong = sketch.add_point2d("wrong", real(2), real(4));
    let x = sketch.add_point2d("x", real(5), real(0));
    let line = sketch.add_line_segment2("x axis", a0, x);
    let three = sketch.add_distance("three", real(3));

    let ratio_report = sketch_distance_builders::length_ratio_lines2(
        &mut sketch,
        "two to one",
        long,
        short,
        real(2),
        real(1),
    );
    let point_line_report = sketch_distance_builders::point_line_distance2(
        &mut sketch,
        "point line",
        near,
        line,
        three,
    );
    sketch.add_length_ratio_lines2("violated ratio", short, long, real(2), real(1));
    sketch.add_point_line_distance2("violated point line", wrong, line, three);

    assert_eq!(
        ratio_report.strategy,
        SketchResidualStrategy::SquaredLineLengthRatio
    );
    assert_eq!(
        point_line_report.strategy,
        SketchResidualStrategy::SquaredPointLineDistance
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::SquaredLineLengthRatio)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredPointLineDistance)
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_length_ratio_and_point_line_distance_report_bad_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    let distance = sketch.add_distance("distance", real(1));
    sketch.add_length_ratio_lines2("negative ratio", line, line, real(-1), real(1));
    sketch.add_length_ratio_lines2("zero denominator", line, line, real(1), real(0));
    sketch.add_length_ratio_lines2(
        "missing line",
        line,
        SketchEntityHandle(999),
        real(1),
        real(1),
    );
    sketch.add_length_ratio_lines2("wrong line", line, p, real(1), real(1));
    sketch.add_point_line_distance2("missing point", SketchEntityHandle(999), line, distance);
    sketch.add_point_line_distance2("wrong point", line, line, distance);
    sketch.add_point_line_distance2("wrong distance", p, line, line);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 7);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[4].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[5].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "point"
        }
    );
    assert_eq!(
        lowered.rows[6].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "distance"
        }
    );
}

#[test]
fn sketch_length_difference_relations_lower_to_exact_polynomial_and_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let long_end = sketch.add_point2d("long end", real(7), real(0));
    let short_end = sketch.add_point2d("short end", real(4), real(0));
    let wrong_end = sketch.add_point2d("wrong end", real(6), real(0));
    let longer = sketch.add_line_segment2("longer", origin, long_end);
    let shorter = sketch.add_line_segment2("shorter", origin, short_end);
    let wrong = sketch.add_line_segment2("wrong", origin, wrong_end);
    let three = sketch.add_distance("three", real(3));
    let two = sketch.add_distance("two", real(2));
    let length_difference = sketch_distance_builders::length_difference_lines2(
        &mut sketch,
        "length difference",
        longer,
        shorter,
        three,
    );
    sketch.add_length_difference_lines2("swapped branch", shorter, longer, three);
    sketch.add_length_difference_lines2("wrong difference", longer, wrong, three);
    sketch.add_length_difference_lines2("different distance", longer, wrong, two);

    assert_eq!(
        length_difference.strategy,
        SketchResidualStrategy::SquaredLineLengthDifference
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 8);
    assert_eq!(lowered.rows.len(), 8);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::SquaredLineLengthDifference)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[6].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_length_difference_relations_report_bad_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    let distance = sketch.add_distance("distance", real(1));
    sketch.add_length_difference_lines2("missing longer", SketchEntityHandle(999), line, distance);
    sketch.add_length_difference_lines2("wrong shorter", line, p, distance);
    sketch.add_length_difference_lines2("wrong distance", line, line, line);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "distance"
        }
    );
}

#[test]
fn sketch_equal_point_line_distance_relations_lower_to_exact_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let x_axis_end = sketch.add_point2d("x axis end", real(5), real(0));
    let three_line_end = sketch.add_point2d("three line end", real(3), real(0));
    let x_axis = sketch.add_line_segment2("x axis", origin, x_axis_end);
    let length_three = sketch.add_line_segment2("length three", origin, three_line_end);
    let distance_point = sketch.add_point2d("distance point", real(2), real(3));
    let other_line_start = sketch.add_point2d("other line start", real(0), real(10));
    let other_line_end = sketch.add_point2d("other line end", real(5), real(10));
    let other_line = sketch.add_line_segment2("other line", other_line_start, other_line_end);
    let equal_point = sketch.add_point2d("equal point", real(2), real(13));
    let wrong_point = sketch.add_point2d("wrong point", real(2), real(14));

    let length_report = sketch_distance_builders::equal_length_point_line_distance2(
        &mut sketch,
        "length equals distance",
        length_three,
        distance_point,
        x_axis,
    );
    let equal_distance_report = sketch_distance_builders::equal_point_line_distances2(
        &mut sketch,
        "equal point-line distances",
        distance_point,
        x_axis,
        equal_point,
        other_line,
    );
    sketch.add_equal_length_point_line_distance2(
        "violated length distance",
        x_axis,
        distance_point,
        other_line,
    );
    sketch.add_equal_point_line_distances2(
        "violated point-line distances",
        distance_point,
        x_axis,
        wrong_point,
        other_line,
    );

    assert_eq!(
        length_report.strategy,
        SketchResidualStrategy::SquaredLineLengthPointLineDistance
    );
    assert_eq!(
        equal_distance_report.strategy,
        SketchResidualStrategy::SquaredEqualPointLineDistances
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::SquaredLineLengthPointLineDistance)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredEqualPointLineDistances)
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
}

#[test]
fn sketch_equal_point_line_distance_relations_report_bad_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let p = sketch.add_point2d("p", real(0), real(0));
    let q = sketch.add_point2d("q", real(1), real(0));
    let line = sketch.add_line_segment2("line", p, q);
    sketch.add_equal_length_point_line_distance2(
        "missing length line",
        SketchEntityHandle(999),
        p,
        line,
    );
    sketch.add_equal_length_point_line_distance2("wrong point", line, line, line);
    sketch.add_equal_length_point_line_distance2("wrong distance line", line, p, p);
    sketch.add_equal_point_line_distances2(
        "missing first point",
        SketchEntityHandle(999),
        line,
        p,
        line,
    );
    sketch.add_equal_point_line_distances2("wrong first line", p, p, q, line);
    sketch.add_equal_point_line_distances2("wrong second point", p, line, line, line);
    sketch.add_equal_point_line_distances2("wrong second line", p, line, q, q);

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 7);
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "point"
        }
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
    assert_eq!(
        lowered.rows[4].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: p,
            expected: "2D line segment"
        }
    );
    assert_eq!(
        lowered.rows[5].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: line,
            expected: "point"
        }
    );
    assert_eq!(
        lowered.rows[6].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: q,
            expected: "2D line segment"
        }
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
fn sketch_workplane_frame_reports_exact_quaternion_axes_and_lift_project() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(10), real(20), real(30));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let point2 = sketch.add_point2d("uv", real(3), real(4));
    let point3 = sketch.add_point3d("xyz", real(13), real(24), real(30));

    let frame = build_sketch_workplane_frame(&sketch, workplane);

    assert_eq!(frame.status, SketchWorkplaneFrameStatus::Certified);
    assert_eq!(
        frame.origin_coordinates,
        Some([real(10), real(20), real(30)])
    );
    assert_eq!(frame.u_axis, Some([real(1), real(0), real(0)]));
    assert_eq!(frame.v_axis, Some([real(0), real(1), real(0)]));
    assert_eq!(frame.n_axis, Some([real(0), real(0), real(1)]));

    let lift = lift_sketch_point2_to_workplane3(&sketch, workplane, point2);
    assert_eq!(lift.status, SketchWorkplaneFrameStatus::Certified);
    assert_eq!(
        lift.lifted_coordinates,
        Some([real(13), real(24), real(30)])
    );

    let project = project_sketch_point3_to_workplane2(&sketch, workplane, point3);
    assert_eq!(project.status, SketchWorkplaneFrameStatus::Certified);
    assert_eq!(project.projected_coordinates, Some([real(3), real(4)]));
}

#[test]
fn sketch_projected_distance_lowers_unit_guard_and_exact_projection_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(10), real(20), real(30));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", real(10), real(20), real(30));
    let b = sketch.add_point3d("b", real(13), real(24), real(99));
    let distance = sketch.add_distance("projected five", real(5));
    let constraint =
        sketch.add_projected_point_point_distance("projected distance", workplane, a, b, distance);

    let report = sketch.lower_to_problem();

    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 2);
    assert_eq!(
        report
            .rows
            .iter()
            .map(|row| row.constraint)
            .collect::<Vec<_>>(),
        vec![constraint, constraint]
    );
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(certification.all_satisfied());
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(constraint);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedDistancePolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::TrueProjectedDistanceProposal
    );
    assert_eq!(forms.forms[2].role, SketchResidualFormRole::ProposalOnly);
}

#[test]
fn sketch_projected_equal_point_point_distances3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(10), real(20), real(30));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", real(10), real(20), real(30));
    let b = sketch.add_point3d("b", real(13), real(24), real(99));
    let c = sketch.add_point3d("c", real(-10), real(7), real(-88));
    let d = sketch.add_point3d("d", real(-7), real(11), real(12));
    let bad = sketch.add_point3d("bad", real(-6), real(11), real(12));
    let valid = sketch_distance_builders::projected_equal_point_point_distances3(
        &mut sketch,
        "projected equal point distances",
        workplane,
        a,
        b,
        c,
        d,
    );
    sketch.add_projected_equal_point_point_distances3(
        "violated projected equal point distances",
        workplane,
        a,
        b,
        c,
        bad,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedEqualPointPointDistances
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedEqualPointPointDistances)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedEqualPointPointDistances)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedEqualPointPointDistancesPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);

    let wrong_2d = sketch.add_point2d("wrong 2d", real(0), real(0));
    let wrong = sketch.add_projected_equal_point_point_distances3(
        "wrong point",
        workplane,
        a,
        wrong_2d,
        c,
        d,
    );
    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(!wrong_forms.diagnostics.is_empty());
}

#[test]
fn sketch_projected_distance_ranges_lower_to_exact_squared_inequalities() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(10), real(20), real(30));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", real(10), real(20), real(30));
    let b = sketch.add_point3d("b", real(13), real(24), real(99));
    let bad = sketch.add_point3d("bad", real(20), real(20), real(-7));
    let range = sketch_distance_builders::projected_point_point_distance_range(
        &mut sketch,
        "projected clearance window",
        workplane,
        a,
        b,
        Some(real(4)),
        Some(real(6)),
    );
    sketch.add_projected_point_point_distance_range(
        "violated projected clearance window",
        workplane,
        a,
        bad,
        Some(real(0)),
        Some(real(6)),
    );

    let report = sketch.lower_to_problem();

    assert_eq!(range.family, hypersolve::SketchConstraintFamily::Distance);
    assert_eq!(
        range.strategy,
        SketchResidualStrategy::BoundedSquaredProjectedDistance
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 6);
    assert_eq!(
        report.problem.constraints[1].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        report.problem.constraints[2].kind,
        ConstraintKind::LessOrEqual
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedDistance)
    );
    assert_eq!(
        report.rows[5].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(range.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::BoundedSquaredProjectedDistancePolynomial
    );
}

#[test]
fn sketch_projected_distance_ranges_report_invalid_empty_and_stale_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", real(0), real(0), real(0));
    let b = sketch.add_point3d("b", real(1), real(0), real(5));
    sketch.add_projected_point_point_distance_range(
        "negative lower",
        workplane,
        a,
        b,
        Some(real(-1)),
        None,
    );
    sketch.add_projected_point_point_distance_range(
        "inverted",
        workplane,
        a,
        b,
        Some(real(6)),
        Some(real(4)),
    );
    sketch.add_projected_point_point_distance_range("empty", workplane, a, b, None, None);
    sketch.add_projected_point_point_distance_range(
        "stale",
        workplane,
        a,
        SketchEntityHandle(999),
        Some(real(0)),
        Some(real(2)),
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedDistance)
    );
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
}

#[test]
fn sketch_projected_distance_reports_bad_workplanes_and_nonunit_frames() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let nonunit = sketch.add_normal3d("nonunit", real(2), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, nonunit);
    let a = sketch.add_point3d("a", real(0), real(0), real(0));
    let b = sketch.add_point3d("b", real(3), real(0), real(7));
    let distance = sketch.add_distance("d", real(3));
    sketch.add_projected_point_point_distance("nonunit projected", workplane, a, b, distance);

    let report = sketch.lower_to_problem();
    assert!(report.all_generated());
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let wrong_workplane = sketch.add_point2d("not a workplane", real(0), real(0));
    sketch.add_projected_point_point_distance(
        "wrong workplane projected",
        wrong_workplane,
        a,
        b,
        distance,
    );
    let wrong = sketch.lower_to_problem();
    assert!(wrong.rows.iter().any(|row| {
        matches!(
            row.status,
            SketchGeneratedRowStatus::WrongEntityKind {
                handle,
                expected: "workplane"
            } if handle == wrong_workplane
        )
    }));
}

#[test]
fn sketch_projected_point_line_distance_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(10));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let point = sketch.add_point3d("point", real(2), real(3), real(17));
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(-5));
    let line_end = sketch.add_point3d("line end", real(4), real(0), real(99));
    let bad_point = sketch.add_point3d("bad point", real(2), real(4), real(17));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let distance = sketch.add_distance("distance three", real(3));
    let valid = sketch_distance_builders::projected_point_line_distance(
        &mut sketch,
        "projected point-line",
        workplane,
        point,
        line,
        distance,
    );
    sketch.add_projected_point_line_distance(
        "bad projected point-line",
        workplane,
        bad_point,
        line,
        distance,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.family,
        hypersolve::sketch_builders::SketchConstraintFamily::Distance
    );
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedPointLineDistance
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointLineDistance)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointLineDistance)
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
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 4);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedPointLineDistancePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedPointLineSignedDistancePositiveProposal
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ProjectedPointLineSignedDistanceNegativeProposal
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
    assert_eq!(forms.forms[2].role, SketchResidualFormRole::ProposalOnly);

    let wrong_line = sketch.add_line_segment2("wrong 2d line", line_start, line_end);
    let bad = sketch.add_projected_point_line_distance(
        "wrong line family",
        workplane,
        point,
        wrong_line,
        distance,
    );
    let bad_forms = sketch.residual_forms_for_constraint(bad);
    assert_eq!(bad_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(!bad_forms.diagnostics.is_empty());
}

#[test]
fn sketch_projected_point_line_distance_ranges_lower_to_exact_inequalities() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(10));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let point = sketch.add_point3d("point", real(2), real(3), real(17));
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(-5));
    let line_end = sketch.add_point3d("line end", real(4), real(0), real(99));
    let bad_point = sketch.add_point3d("bad point", real(2), real(8), real(17));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let valid = sketch_distance_builders::projected_point_line_distance_range(
        &mut sketch,
        "projected point-line clearance window",
        workplane,
        point,
        line,
        Some(real(2)),
        Some(real(4)),
    );
    sketch.add_projected_point_line_distance_range(
        "violated projected point-line clearance window",
        workplane,
        bad_point,
        line,
        Some(real(0)),
        Some(real(4)),
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 6);
    assert_eq!(
        report.problem.constraints[1].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        report.problem.constraints[2].kind,
        ConstraintKind::LessOrEqual
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance)
    );
    assert_eq!(
        report.rows[5].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::BoundedSquaredProjectedPointLineDistancePolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_point_line_distance_ranges_report_invalid_empty_and_stale_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let point = sketch.add_point3d("point", real(0), real(3), real(9));
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(0));
    let line_end = sketch.add_point3d("line end", real(4), real(0), real(7));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    sketch.add_projected_point_line_distance_range(
        "negative lower",
        workplane,
        point,
        line,
        Some(real(-1)),
        None,
    );
    sketch.add_projected_point_line_distance_range(
        "inverted",
        workplane,
        point,
        line,
        Some(real(6)),
        Some(real(4)),
    );
    sketch.add_projected_point_line_distance_range("empty", workplane, point, line, None, None);
    sketch.add_projected_point_line_distance_range(
        "stale",
        workplane,
        point,
        SketchEntityHandle(999),
        Some(real(0)),
        Some(real(2)),
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance)
    );
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
}

#[test]
fn sketch_projected_equal_length_lines3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a0 = sketch.add_point3d("a0", real(0), real(0), real(7));
    let a1 = sketch.add_point3d("a1", real(3), real(4), real(11));
    let b0 = sketch.add_point3d("b0", real(10), real(10), real(-2));
    let b1 = sketch.add_point3d("b1", real(15), real(10), real(99));
    let bad0 = sketch.add_point3d("bad0", real(0), real(0), real(0));
    let bad1 = sketch.add_point3d("bad1", real(6), real(0), real(0));
    let a = sketch.add_line_segment3("a", a0, a1);
    let b = sketch.add_line_segment3("b", b0, b1);
    let bad = sketch.add_line_segment3("bad", bad0, bad1);
    let valid = sketch_distance_builders::projected_equal_length_lines3(
        &mut sketch,
        "projected equal length",
        workplane,
        a,
        b,
    );
    sketch.add_projected_equal_length_lines3("projected unequal length", workplane, a, bad);

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedLineLengthEquality
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthEquality)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthEquality)
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
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedLineLengthEqualityPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);

    let p2a = sketch.add_point2d("wrong 2d a", real(0), real(0));
    let p2b = sketch.add_point2d("wrong 2d b", real(1), real(0));
    let wrong_line = sketch.add_line_segment2("wrong 2d line", p2a, p2b);
    let wrong = sketch.add_projected_equal_length_lines3("wrong family", workplane, wrong_line, b);
    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(!wrong_forms.diagnostics.is_empty());
}

#[test]
fn sketch_projected_line_length_ranges_lower_to_exact_inequalities() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(-5));
    let line_end = sketch.add_point3d("line end", real(3), real(4), real(99));
    let bad_start = sketch.add_point3d("bad start", real(0), real(0), real(0));
    let bad_end = sketch.add_point3d("bad end", real(9), real(0), real(17));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let bad_line = sketch.add_line_segment3("bad line", bad_start, bad_end);
    let valid = sketch_distance_builders::projected_line_length_range3(
        &mut sketch,
        "projected line length window",
        workplane,
        line,
        Some(real(4)),
        Some(real(6)),
    );
    sketch.add_projected_line_length_range3(
        "violated projected line length window",
        workplane,
        bad_line,
        Some(real(0)),
        Some(real(6)),
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::BoundedSquaredProjectedLineLength
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 6);
    assert_eq!(
        report.problem.constraints[1].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        report.problem.constraints[2].kind,
        ConstraintKind::LessOrEqual
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedLineLength)
    );
    assert_eq!(
        report.rows[5].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedLineLength)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::BoundedSquaredProjectedLineLengthPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_line_length_ranges_report_invalid_empty_and_stale_inputs() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(0));
    let line_end = sketch.add_point3d("line end", real(3), real(4), real(7));
    let line = sketch.add_line_segment3("line", line_start, line_end);
    sketch.add_projected_line_length_range3(
        "negative lower",
        workplane,
        line,
        Some(real(-1)),
        None,
    );
    sketch.add_projected_line_length_range3(
        "inverted",
        workplane,
        line,
        Some(real(6)),
        Some(real(4)),
    );
    sketch.add_projected_line_length_range3("empty", workplane, line, None, None);
    sketch.add_projected_line_length_range3(
        "stale",
        workplane,
        SketchEntityHandle(999),
        Some(real(0)),
        Some(real(2)),
    );

    let lowered = sketch.lower_to_problem();

    assert_eq!(lowered.problem.constraints.len(), 0);
    assert_eq!(lowered.rows.len(), 4);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::BoundedSquaredProjectedLineLength)
    );
    assert_eq!(
        lowered.rows[0].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[1].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        lowered.rows[2].status,
        SketchGeneratedRowStatus::ReferenceOnly
    );
    assert_eq!(
        lowered.rows[3].status,
        SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
    );
}

#[test]
fn sketch_projected_length_ratio_lines3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a0 = sketch.add_point3d("a0", real(0), real(0), real(7));
    let a1 = sketch.add_point3d("a1", real(6), real(8), real(11));
    let b0 = sketch.add_point3d("b0", real(10), real(10), real(-2));
    let b1 = sketch.add_point3d("b1", real(13), real(14), real(99));
    let bad0 = sketch.add_point3d("bad0", real(0), real(0), real(0));
    let bad1 = sketch.add_point3d("bad1", real(6), real(0), real(0));
    let a = sketch.add_line_segment3("a", a0, a1);
    let b = sketch.add_line_segment3("b", b0, b1);
    let bad = sketch.add_line_segment3("bad", bad0, bad1);
    let valid = sketch_distance_builders::projected_length_ratio_lines3(
        &mut sketch,
        "projected ratio",
        workplane,
        a,
        b,
        real(2),
        real(1),
    );
    sketch.add_projected_length_ratio_lines3(
        "violated projected ratio",
        workplane,
        b,
        bad,
        real(2),
        real(1),
    );
    sketch.add_projected_length_ratio_lines3(
        "invalid projected ratio",
        workplane,
        a,
        b,
        real(-1),
        real(1),
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedLineLengthRatio
    );
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio)
    );
    assert_eq!(
        report.rows[4].status,
        SketchGeneratedRowStatus::InvalidExactBound
    );
    assert_eq!(
        report.rows[4].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedLineLengthRatioPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_length_difference_lines3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let long0 = sketch.add_point3d("long0", real(0), real(0), real(7));
    let long1 = sketch.add_point3d("long1", real(6), real(8), real(12));
    let short0 = sketch.add_point3d("short0", real(20), real(20), real(-3));
    let short1 = sketch.add_point3d("short1", real(23), real(24), real(50));
    let bad0 = sketch.add_point3d("bad0", real(0), real(0), real(0));
    let bad1 = sketch.add_point3d("bad1", real(8), real(0), real(0));
    let longer = sketch.add_line_segment3("longer", long0, long1);
    let shorter = sketch.add_line_segment3("shorter", short0, short1);
    let bad = sketch.add_line_segment3("bad", bad0, bad1);
    let difference = sketch.add_distance("difference", real(5));
    let valid = sketch_distance_builders::projected_length_difference_lines3(
        &mut sketch,
        "projected difference",
        workplane,
        longer,
        shorter,
        difference,
    );
    sketch.add_projected_length_difference_lines3(
        "violated projected difference",
        workplane,
        bad,
        shorter,
        difference,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedLineLengthDifference
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 6);
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthDifference)
    );
    assert_eq!(
        report.problem.constraints[2].kind,
        ConstraintKind::GreaterOrEqual
    );
    assert_eq!(
        report.rows[4].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthDifference)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedLineLengthDifferencePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedLineLengthDifferenceBranchPredicate
    );
    assert_eq!(forms.forms[2].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_equal_length_point_line_distance3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let len0 = sketch.add_point3d("len0", real(0), real(20), real(-7));
    let len1 = sketch.add_point3d("len1", real(5), real(20), real(13));
    let bad_len0 = sketch.add_point3d("bad_len0", real(0), real(30), real(7));
    let bad_len1 = sketch.add_point3d("bad_len1", real(4), real(30), real(13));
    let distance_point = sketch.add_point3d("distance_point", real(0), real(5), real(99));
    let distance0 = sketch.add_point3d("distance0", real(-10), real(0), real(-3));
    let distance1 = sketch.add_point3d("distance1", real(10), real(0), real(21));
    let length_line = sketch.add_line_segment3("length_line", len0, len1);
    let bad_length_line = sketch.add_line_segment3("bad_length_line", bad_len0, bad_len1);
    let distance_line = sketch.add_line_segment3("distance_line", distance0, distance1);
    let valid = sketch_distance_builders::projected_equal_length_point_line_distance3(
        &mut sketch,
        "projected length equals point-line distance",
        workplane,
        length_line,
        distance_point,
        distance_line,
    );
    sketch.add_projected_equal_length_point_line_distance3(
        "violated projected length equals point-line distance",
        workplane,
        bad_length_line,
        distance_point,
        distance_line,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedLineLengthPointLineDistancePolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_equal_point_line_distances3_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a_point = sketch.add_point3d("a_point", real(0), real(5), real(77));
    let a0 = sketch.add_point3d("a0", real(-10), real(0), real(-2));
    let a1 = sketch.add_point3d("a1", real(10), real(0), real(9));
    let b_point = sketch.add_point3d("b_point", real(1), real(25), real(-99));
    let b0 = sketch.add_point3d("b0", real(-8), real(20), real(12));
    let b1 = sketch.add_point3d("b1", real(8), real(20), real(30));
    let bad_point = sketch.add_point3d("bad_point", real(1), real(26), real(-99));
    let a_line = sketch.add_line_segment3("a_line", a0, a1);
    let b_line = sketch.add_line_segment3("b_line", b0, b1);
    let valid = sketch_distance_builders::projected_equal_point_line_distances3(
        &mut sketch,
        "projected equal point-line distances",
        workplane,
        a_point,
        a_line,
        b_point,
        b_line,
    );
    sketch.add_projected_equal_point_line_distances3(
        "violated projected equal point-line distances",
        workplane,
        a_point,
        a_line,
        bad_point,
        b_line,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::SquaredProjectedEqualPointLineDistances
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 4);
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedEqualPointLineDistances)
    );
    assert_eq!(
        report.rows[3].strategy,
        Some(SketchResidualStrategy::SquaredProjectedEqualPointLineDistances)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedEqualPointLineDistancesPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
}

#[test]
fn sketch_projected_oriented_angle_lowers_exact_workplane_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(10));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point3d("p0", real(0), real(0), real(0));
    let x = sketch.add_point3d("x", real(3), real(0), real(7));
    let y = sketch.add_point3d("y", real(0), real(4), real(-9));
    let sx = sketch.add_point3d("sx", real(6), real(0), real(11));
    let sy = sketch.add_point3d("sy", real(0), real(8), real(-3));
    let neg_y = sketch.add_point3d("neg y", real(0), real(-8), real(5));
    let a = sketch.add_line_segment3("a", p0, x);
    let b = sketch.add_line_segment3("b", p0, y);
    let c = sketch.add_line_segment3("c", p0, sx);
    let d = sketch.add_line_segment3("d", p0, sy);
    let bad_d = sketch.add_line_segment3("bad d", p0, neg_y);
    let valid = sketch_angle_builders::projected_equal_oriented_angle_lines3(
        &mut sketch,
        "projected oriented angle",
        workplane,
        a,
        b,
        c,
        d,
    );
    sketch.add_projected_equal_oriented_angle_lines3(
        "bad projected oriented angle",
        workplane,
        a,
        b,
        c,
        bad_d,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        valid.family,
        hypersolve::sketch_builders::SketchConstraintFamily::Angle
    );
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedOrientedAngleEquality
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 6);
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedOrientedAngleEquality)
    );
    assert_eq!(
        report.rows[2].strategy,
        Some(SketchResidualStrategy::ProjectedOrientedAngleEquality)
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
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedOrientedAngleVectorCollinearityPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedOrientedAngleSameBranchPredicate
    );
    assert_eq!(forms.forms[2].role, SketchResidualFormRole::ExactProof);

    let wrong_2d_line = sketch.add_line_segment2("wrong 2d line", p0, x);
    let bad = sketch.add_projected_equal_oriented_angle_lines3(
        "wrong projected angle",
        workplane,
        wrong_2d_line,
        b,
        c,
        d,
    );
    let bad_forms = sketch.residual_forms_for_constraint(bad);
    assert_eq!(bad_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(!bad_forms.diagnostics.is_empty());
}

#[test]
fn sketch_projected_line_orientation_lowers_exact_workplane_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point3d("p0", real(0), real(0), real(0));
    let px = sketch.add_point3d("px", real(5), real(0), real(11));
    let p2 = sketch.add_point3d("p2", real(1), real(2), real(-7));
    let p2x = sketch.add_point3d("p2x", real(11), real(2), real(17));
    let p2negx = sketch.add_point3d("p2negx", real(-9), real(2), real(19));
    let py = sketch.add_point3d("py", real(0), real(7), real(23));
    let p2y = sketch.add_point3d("p2y", real(1), real(9), real(31));
    let x = sketch.add_line_segment3("x", p0, px);
    let shifted_x = sketch.add_line_segment3("shifted x", p2, p2x);
    let reversed_x = sketch.add_line_segment3("reversed x", p2, p2negx);
    let y = sketch.add_line_segment3("y", p0, py);
    let shifted_y = sketch.add_line_segment3("shifted y", p2, p2y);
    let parallel = hypersolve::sketch_orientation_builders::projected_parallel_lines3(
        &mut sketch,
        "projected parallel",
        workplane,
        x,
        shifted_x,
    );
    sketch.add_projected_parallel_lines3("violated projected parallel", workplane, x, y);
    let perpendicular = hypersolve::sketch_orientation_builders::projected_perpendicular_lines3(
        &mut sketch,
        "projected perpendicular",
        workplane,
        x,
        shifted_y,
    );
    sketch.add_projected_perpendicular_lines3(
        "violated projected perpendicular",
        workplane,
        x,
        shifted_x,
    );
    let same_direction = hypersolve::sketch_orientation_builders::projected_same_direction_lines3(
        &mut sketch,
        "projected same direction",
        workplane,
        x,
        shifted_x,
    );
    sketch.add_projected_same_direction_lines3(
        "violated projected same direction",
        workplane,
        x,
        reversed_x,
    );

    let report = sketch.lower_to_problem();

    assert_eq!(
        parallel.family,
        hypersolve::sketch_builders::SketchConstraintFamily::Orientation
    );
    assert_eq!(
        parallel.strategy,
        SketchResidualStrategy::ProjectedDirectionCrossProduct
    );
    assert_eq!(
        perpendicular.strategy,
        SketchResidualStrategy::ProjectedDirectionDotProduct
    );
    assert_eq!(
        same_direction.strategy,
        SketchResidualStrategy::ProjectedDirectionSameOrientation
    );
    assert!(report.all_generated());
    assert_eq!(report.problem.constraints.len(), 14);
    assert_eq!(
        report.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        report.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedDirectionCrossProduct)
    );
    assert_eq!(
        report.rows[5].strategy,
        Some(SketchResidualStrategy::ProjectedDirectionDotProduct)
    );
    assert_eq!(
        report.rows[9].strategy,
        Some(SketchResidualStrategy::ProjectedDirectionSameOrientation)
    );
    assert_eq!(
        report.problem.constraints[10].kind,
        ConstraintKind::GreaterOrEqual
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&report.problem),
        &context_from_problem(&report.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[7].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[9].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[10].status,
        CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));
    assert!(matches!(
        certification.rows[12].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[13].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let parallel_forms = sketch.residual_forms_for_constraint(parallel.handle);
    assert_eq!(parallel_forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(parallel_forms.forms.len(), 2);
    assert_eq!(
        parallel_forms.forms[1].kind,
        SketchResidualFormKind::ProjectedDirectionCrossProductPolynomial
    );
    assert_eq!(
        parallel_forms.forms[1].role,
        SketchResidualFormRole::ExactProof
    );

    let perpendicular_forms = sketch.residual_forms_for_constraint(perpendicular.handle);
    assert_eq!(
        perpendicular_forms.forms[1].kind,
        SketchResidualFormKind::ProjectedDirectionDotProductPolynomial
    );

    let same_direction_forms = sketch.residual_forms_for_constraint(same_direction.handle);
    assert_eq!(
        same_direction_forms.status,
        SketchResidualFormsStatus::Generated
    );
    assert_eq!(same_direction_forms.forms.len(), 3);
    assert_eq!(
        same_direction_forms.forms[1].kind,
        SketchResidualFormKind::ProjectedDirectionCrossProductPolynomial
    );
    assert_eq!(
        same_direction_forms.forms[2].kind,
        SketchResidualFormKind::ProjectedDirectionSameOrientationPredicate
    );
    assert_eq!(
        same_direction_forms.forms[2].role,
        SketchResidualFormRole::ExactProof
    );

    let p2a = sketch.add_point2d("wrong 2d a", real(0), real(0));
    let p2b = sketch.add_point2d("wrong 2d b", real(1), real(0));
    let wrong_line = sketch.add_line_segment2("wrong 2d line", p2a, p2b);
    let wrong =
        sketch.add_projected_parallel_lines3("wrong projected parallel", workplane, wrong_line, x);
    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert!(!wrong_forms.diagnostics.is_empty());
}

#[test]
fn sketch_line_arc_length_replays_endpoint_radius_and_transcendental_length() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let start = sketch.add_point2d("start", real(1), real(0));
    let end = sketch.add_point2d("end", real(-1), real(0));
    let radius = sketch.add_distance("radius", real(1));
    let arc = sketch.add_arc_of_circle2("half arc", origin, start, end, radius);
    let line_start = sketch.add_point2d("line start", real(0), real(0));
    let line_end = sketch.add_point2d("line end", Real::pi(), real(0));
    let valid_line = sketch.add_line_segment2("pi line", line_start, line_end);
    let short_end = sketch.add_point2d("short end", real(2), real(0));
    let short_line = sketch.add_line_segment2("short line", line_start, short_end);
    let valid = sketch.add_equal_line_arc_length2("line equals half arc", valid_line, arc);
    sketch.add_equal_line_arc_length2("short line rejects half arc", short_line, arc);
    let wrong = sketch.add_constraint(
        "wrong arc kind",
        SketchConstraintKind::EqualLineArcLength2 {
            line: valid_line,
            arc: valid_line,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(
        lowered
            .rows
            .iter()
            .filter(|row| row.strategy == Some(SketchResidualStrategy::LineArcLength))
            .count(),
        6
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: valid_line,
            expected: "2D circular arc",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[5].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::LineArcLengthEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::LineArcLengthEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::LineArcLengthTranscendentalEquality
    );
    assert!(forms.forms.iter().all(|form| {
        form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::LineArcLength)
    }));

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
    assert_eq!(
        wrong_forms.diagnostics[0].status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: valid_line,
            expected: "2D circular arc",
        }
    );
}

#[test]
fn sketch_line_arc_sweep_length_replays_major_minor_orientation_branches() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let start = sketch.add_point2d("start", real(1), real(0));
    let end = sketch.add_point2d("end", real(0), real(1));
    let radius = sketch.add_distance("radius", real(1));
    let arc = sketch.add_arc_of_circle2("quarter arc", origin, start, end, radius);
    let line_start = sketch.add_point2d("line start", real(0), real(0));
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());
    let minor_end = sketch.add_point2d("minor end", Real::pi() * half, real(0));
    let major_end = sketch.add_point2d("major end", Real::pi() * three_halves, real(0));
    let invalid_end = sketch.add_point2d("invalid branch end", Real::pi() * real(2), real(0));
    let minor_line = sketch.add_line_segment2("minor line", line_start, minor_end);
    let major_line = sketch.add_line_segment2("major line", line_start, major_end);
    let invalid_line = sketch.add_line_segment2("invalid branch line", line_start, invalid_end);
    let minor = sketch.add_equal_line_arc_sweep_length2(
        "ccw minor quarter",
        minor_line,
        arc,
        SketchArcLengthSweep::CounterClockwiseMinor,
    );
    let major = sketch.add_equal_line_arc_sweep_length2(
        "cw major quarter",
        major_line,
        arc,
        SketchArcLengthSweep::ClockwiseMajor,
    );
    sketch.add_equal_line_arc_sweep_length2(
        "wrong cw minor branch",
        invalid_line,
        arc,
        SketchArcLengthSweep::ClockwiseMinor,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 12);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::LineArcSweepLength)
            && row.status == SketchGeneratedRowStatus::Generated
    }));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[7].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[10].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let minor_forms = sketch.residual_forms_for_constraint(minor);
    assert_eq!(minor_forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(minor_forms.forms.len(), 4);
    assert_eq!(
        minor_forms.forms[2].kind,
        SketchResidualFormKind::LineArcSweepLengthBranchPredicate
    );
    assert_eq!(
        minor_forms.forms[3].kind,
        SketchResidualFormKind::LineArcSweepLengthTranscendentalEquality
    );
    assert!(minor_forms.forms.iter().all(|form| {
        form.role == SketchResidualFormRole::ExactProof
            && form.strategy == Some(SketchResidualStrategy::LineArcSweepLength)
    }));

    let major_forms = sketch.residual_forms_for_constraint(major);
    assert_eq!(
        major_forms.forms[3].kind,
        SketchResidualFormKind::LineArcSweepLengthTranscendentalEquality
    );
}

#[test]
fn sketch_projected_line_arc_sweep_length_replays_workplane_and_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(0), real(0), real(0));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let start = sketch.add_point2d("start", real(1), real(0));
    let end = sketch.add_point2d("end", real(0), real(1));
    let radius = sketch.add_distance("radius", real(1));
    let arc = sketch.add_arc_of_circle2("quarter arc", origin, start, end, radius);
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());
    let line_start = sketch.add_point3d("line start", real(0), real(0), real(5));
    let line_end = sketch.add_point3d("line end", Real::pi() * three_halves, real(0), real(7));
    let line = sketch.add_line_segment3("projected major line", line_start, line_end);
    let bad_line_end = sketch.add_point3d("bad line end", Real::pi(), real(0), real(7));
    let bad_line = sketch.add_line_segment3("bad projected line", line_start, bad_line_end);
    let valid = sketch.add_projected_equal_line_arc_sweep_length3(
        "projected line equals cw major arc",
        workplane,
        line,
        arc,
        SketchArcLengthSweep::ClockwiseMajor,
    );
    sketch.add_projected_equal_line_arc_sweep_length3(
        "projected line wrong branch and length",
        workplane,
        bad_line,
        arc,
        SketchArcLengthSweep::ClockwiseMinor,
    );
    let wrong = sketch.add_constraint(
        "wrong projected line kind",
        SketchConstraintKind::ProjectedEqualLineArcSweepLength3 {
            workplane,
            line: start,
            arc,
            sweep: SketchArcLengthSweep::ClockwiseMajor,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 10);
    assert_eq!(
        lowered
            .rows
            .iter()
            .filter(|row| {
                row.strategy == Some(SketchResidualStrategy::ProjectedLineArcSweepLength)
                    || row.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
            })
            .count(),
        10
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: start,
            expected: "3D line segment",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[4].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[8].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));
    assert!(matches!(
        certification.rows[9].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::LineArcSweepLengthBranchPredicate
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedLineArcSweepLengthTranscendentalEquality
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_projected_point_on_circle_replays_workplane_and_incidence_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let center = sketch.add_point2d("center", real(0), real(0));
    let radius = sketch.add_distance("radius", real(5));
    let circle = sketch.add_circle2("circle", center, radius);
    let valid_point = sketch.add_point3d("valid projected point", real(4), real(6), real(11));
    let invalid_point = sketch.add_point3d("invalid projected point", real(7), real(2), real(11));
    let valid = sketch.add_projected_point_on_circle3(
        "projected point on circle",
        workplane,
        valid_point,
        circle,
    );
    sketch.add_projected_point_on_circle3(
        "bad projected point on circle",
        workplane,
        invalid_point,
        circle,
    );
    let wrong = sketch.add_constraint(
        "wrong projected circle kind",
        SketchConstraintKind::ProjectedPointOnCircle3 {
            workplane,
            point: valid_point,
            circle: radius,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(
        lowered
            .rows
            .iter()
            .filter(|row| {
                row.strategy == Some(SketchResidualStrategy::ProjectedSquaredIncidence)
                    || row.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
            })
            .count(),
        4
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: radius,
            expected: "circle",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedCircleIncidencePolynomial
    );
    assert!(
        forms
            .forms
            .iter()
            .all(|form| form.role == SketchResidualFormRole::ExactProof)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_projected_point_on_line_replays_workplane_and_collinearity_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let start = sketch.add_point3d("start", real(2), real(4), real(-11));
    let end = sketch.add_point3d("end", real(8), real(7), real(19));
    let line = sketch.add_line_segment3("line", start, end);
    let point = sketch.add_point3d("point", real(6), real(6), real(99));
    let off_line = sketch.add_point3d("off line", real(6), real(7), real(99));
    let point2 = sketch.add_point2d("not a 3D line", real(0), real(0));
    let valid = sketch_incidence_builders::projected_point_on_line3(
        &mut sketch,
        "projected point on line",
        workplane,
        point,
        line,
    );
    sketch.add_projected_point_on_line3("off projected line", workplane, off_line, line);
    let wrong =
        sketch.add_projected_point_on_line3("wrong projected line kind", workplane, point, point2);

    assert_eq!(valid.family, hypersolve::SketchConstraintFamily::Incidence);
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedPointLineIncidence
    );

    let lowered = sketch.lower_to_problem();
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );

    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(lowered.rows.len(), 5);
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: point2,
            expected: "3D line segment",
        }
    );
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedPointLineIncidence)
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedPointLineIncidencePolynomial
    );
    assert!(
        forms
            .forms
            .iter()
            .all(|form| form.role == SketchResidualFormRole::ExactProof)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_projected_line_circle_tangent_replays_workplane_and_tangency_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let center = sketch.add_point2d("center", real(0), real(0));
    let radius = sketch.add_distance("radius", real(5));
    let circle = sketch.add_circle2("circle", center, radius);
    let valid_start = sketch.add_point3d("valid start", real(-9), real(7), real(-11));
    let valid_end = sketch.add_point3d("valid end", real(11), real(7), real(23));
    let bad_start = sketch.add_point3d("bad start", real(-9), real(8), real(-11));
    let bad_end = sketch.add_point3d("bad end", real(11), real(8), real(23));
    let valid_line = sketch.add_line_segment3("valid line", valid_start, valid_end);
    let bad_line = sketch.add_line_segment3("bad line", bad_start, bad_end);
    let valid = sketch_tangency_builders::projected_line_circle_tangent3(
        &mut sketch,
        "projected line circle tangent",
        workplane,
        valid_line,
        circle,
    );
    sketch.add_projected_line_circle_tangent3("bad projected tangent", workplane, bad_line, circle);
    let wrong = sketch.add_constraint(
        "wrong projected tangent circle kind",
        SketchConstraintKind::ProjectedLineCircleTangent3 {
            workplane,
            line: valid_line,
            circle: radius,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(
        valid.strategy,
        SketchResidualStrategy::ProjectedLineCircleTangency
    );
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedLineCircleTangency)
    );
    assert_eq!(
        lowered.rows[3].strategy,
        Some(SketchResidualStrategy::ProjectedLineCircleTangency)
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: radius,
            expected: "circle",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid.handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedLineCircleTangencyPolynomial
    );
    assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_point_on_arc_replays_radius_and_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let valid_point = sketch.add_point2d("valid point", real(3), real(4));
    let outside_point = sketch.add_point2d("outside point", real(-3), real(4));
    let valid = sketch.add_point_on_arc2(
        "point on ccw minor arc",
        valid_point,
        arc,
        SketchArcPointSweep::CounterClockwiseMinor,
    );
    sketch.add_point_on_arc2(
        "point on wrong arc branch",
        outside_point,
        arc,
        SketchArcPointSweep::CounterClockwiseMinor,
    );
    let wrong = sketch.add_constraint(
        "wrong arc kind",
        SketchConstraintKind::PointOnArc2 {
            point: valid_point,
            arc: radius,
            sweep: SketchArcPointSweep::CounterClockwiseMinor,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 12);
    assert_eq!(
        lowered
            .rows
            .iter()
            .filter(|row| row.strategy == Some(SketchResidualStrategy::PointArcIncidence))
            .count(),
        12
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: radius,
            expected: "2D circular arc",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[11].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 6);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::ArcIncidenceEndpointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ArcIncidencePointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ArcIncidenceSweepBranchPredicate
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ArcIncidencePointBranchPredicate
    );
    assert!(
        forms
            .forms
            .iter()
            .all(|form| form.role == SketchResidualFormRole::ExactProof)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_projected_point_on_arc_replays_radius_and_branch_rows() {
    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", real(1), real(2), real(3));
    let normal = sketch.add_normal3d("identity normal", real(1), real(0), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let center = sketch.add_point2d("center", real(0), real(0));
    let start = sketch.add_point2d("start", real(5), real(0));
    let end = sketch.add_point2d("end", real(0), real(5));
    let radius = sketch.add_distance("radius", real(5));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius);
    let valid_point = sketch.add_point3d("valid projected point", real(4), real(6), real(11));
    let outside_point = sketch.add_point3d("outside projected point", real(-2), real(6), real(11));
    let valid = sketch.add_projected_point_on_arc3(
        "projected point on ccw minor arc",
        workplane,
        valid_point,
        arc,
        SketchArcPointSweep::CounterClockwiseMinor,
    );
    sketch.add_projected_point_on_arc3(
        "projected point on wrong arc branch",
        workplane,
        outside_point,
        arc,
        SketchArcPointSweep::CounterClockwiseMinor,
    );
    let wrong = sketch.add_constraint(
        "wrong projected arc kind",
        SketchConstraintKind::ProjectedPointOnArc3 {
            workplane,
            point: valid_point,
            arc: radius,
            sweep: SketchArcPointSweep::CounterClockwiseMinor,
        },
        false,
        true,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 14);
    assert_eq!(
        lowered
            .rows
            .iter()
            .filter(|row| {
                row.strategy == Some(SketchResidualStrategy::ProjectedPointArcIncidence)
                    || row.strategy == Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
            })
            .count(),
        14
    );
    assert_eq!(
        lowered.rows.last().unwrap().status,
        SketchGeneratedRowStatus::WrongEntityKind {
            handle: radius,
            expected: "2D circular arc",
        }
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[3].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[13].status,
        CertifiedCandidateStatus::CertifiedViolation { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(valid);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 7);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ProjectedArcIncidencePointRadiusPolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedArcIncidenceSweepBranchPredicate
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ProjectedArcIncidencePointBranchPredicate
    );
    assert_eq!(
        forms.forms[6].kind,
        SketchResidualFormKind::ProjectedArcIncidencePointBranchPredicate
    );
    assert!(
        forms
            .forms
            .iter()
            .all(|form| form.role == SketchResidualFormRole::ExactProof)
    );

    let wrong_forms = sketch.residual_forms_for_constraint(wrong);
    assert_eq!(wrong_forms.status, SketchResidualFormsStatus::InvalidInputs);
}

#[test]
fn sketch_workplane_frame_respects_rotated_unit_quaternion_basis() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(5));
    let normal = sketch.add_normal3d("flip z", real(0), real(1), real(0), real(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let point2 = sketch.add_point2d("uv", real(2), real(3));

    let frame = build_sketch_workplane_frame(&sketch, workplane);
    let lift = lift_sketch_point2_to_workplane3(&sketch, workplane, point2);

    assert_eq!(frame.status, SketchWorkplaneFrameStatus::Certified);
    assert_eq!(frame.u_axis, Some([real(1), real(0), real(0)]));
    assert_eq!(frame.v_axis, Some([real(0), real(-1), real(0)]));
    assert_eq!(frame.n_axis, Some([real(0), real(0), real(-1)]));
    assert_eq!(lift.lifted_coordinates, Some([real(2), real(-3), real(5)]));
}

#[test]
fn sketch_workplane_frame_reports_nonunit_and_invalid_references() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", real(0), real(0), real(0));
    let point2 = sketch.add_point2d("point2", real(0), real(0));
    let distance = sketch.add_distance("distance", real(1));
    let bad_normal = sketch.add_normal3d("bad normal", real(2), real(0), real(0), real(0));
    let nonunit_workplane = sketch.add_workplane("nonunit", origin, bad_normal);
    let wrong_origin_workplane = sketch.add_workplane("wrong origin", point2, bad_normal);
    let wrong_normal_workplane = sketch.add_workplane("wrong normal", origin, distance);

    let nonunit = build_sketch_workplane_frame(&sketch, nonunit_workplane);
    assert_eq!(
        nonunit.status,
        SketchWorkplaneFrameStatus::NonunitNormal {
            squared_norm: real(4)
        }
    );

    let wrong_origin = build_sketch_workplane_frame(&sketch, wrong_origin_workplane);
    assert_eq!(
        wrong_origin.status,
        SketchWorkplaneFrameStatus::WrongEntityKind {
            handle: point2,
            expected: "3D point"
        }
    );

    let wrong_normal = build_sketch_workplane_frame(&sketch, wrong_normal_workplane);
    assert_eq!(
        wrong_normal.status,
        SketchWorkplaneFrameStatus::WrongEntityKind {
            handle: distance,
            expected: "3D normal"
        }
    );

    let missing = build_sketch_workplane_frame(&sketch, SketchEntityHandle(999));
    assert_eq!(
        missing.status,
        SketchWorkplaneFrameStatus::MissingEntity {
            handle: SketchEntityHandle(999)
        }
    );
}

#[test]
fn sketch_entity_domains_certify_unit_normals_radii_tangents_and_arcs() {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", real(0), real(0));
    let same = sketch.add_point2d("same", real(0), real(0));
    let unit_x = sketch.add_point2d("unit x", real(1), real(0));
    let radius = sketch.add_distance("radius", real(2));
    let zero_radius = sketch.add_distance("zero radius", real(0));
    let unit_normal = sketch.add_normal2d("unit normal", real(0), real(1));
    let nonunit_normal = sketch.add_normal3d("bad normal", real(2), real(0), real(0), real(0));
    let line = sketch.add_line_segment2("line", origin, unit_x);
    let zero_tangent = sketch.add_line_segment2("zero tangent", origin, same);
    let origin3 = sketch.add_point3d("origin3", real(0), real(0), real(0));
    let unit_z = sketch.add_point3d("unit z", real(0), real(0), real(1));
    let same3 = sketch.add_point3d("same3", real(0), real(0), real(0));
    let line3 = sketch.add_line_segment3("line3", origin3, unit_z);
    let zero_line3 = sketch.add_line_segment3("zero line3", origin3, same3);
    let arc = sketch.add_arc_of_circle2("arc", origin, origin, unit_x, radius);
    let full_arc = sketch.add_arc_of_circle2("full arc", origin, same, same, radius);
    let circle = sketch.add_circle2("circle", origin, zero_radius);

    assert!(sketch.add_entity_domain(unit_normal, SketchEntityDomain::UnitNormal));
    assert!(sketch.add_entity_domain(nonunit_normal, SketchEntityDomain::UnitNormal));
    assert!(sketch.add_entity_domain(radius, SketchEntityDomain::PositiveRadius));
    assert!(sketch.add_entity_domain(circle, SketchEntityDomain::PositiveRadius));
    assert!(sketch.add_entity_domain(line, SketchEntityDomain::NonzeroLengthLineSegment2));
    assert!(sketch.add_entity_domain(line3, SketchEntityDomain::NonzeroLengthLineSegment3));
    assert!(sketch.add_entity_domain(zero_line3, SketchEntityDomain::NonzeroLengthLineSegment3));
    assert!(
        sketch.add_entity_domain(zero_tangent, SketchEntityDomain::NonzeroTangentLineSegment2,)
    );
    assert!(sketch.add_entity_domain(arc, SketchEntityDomain::NondegenerateArc2));
    assert!(sketch.add_entity_domain(full_arc, SketchEntityDomain::NondegenerateArc2));

    let report = preflight_sketch_entity_domains(&sketch);

    assert_eq!(report.entity_count, sketch.entities().len());
    assert_eq!(report.checks.len(), 10);
    assert_eq!(report.certified_valid_checks, 5);
    assert_eq!(report.certified_invalid_checks, 5);
    assert_eq!(report.unknown_checks, 0);
    assert_eq!(report.invalid_reference_checks, 0);
    assert!(report.has_certified_invalid_domain());
    assert!(!report.all_certified_valid());
    assert!(report.checks.iter().any(|check| {
        check.entity == unit_normal
            && check.kind == SketchEntityDomainKind::UnitNormal
            && check.status == SketchEntityDomainStatus::CertifiedValid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == nonunit_normal
            && check.kind == SketchEntityDomainKind::UnitNormal
            && check.status == SketchEntityDomainStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == circle
            && check.kind == SketchEntityDomainKind::PositiveRadius
            && check.status == SketchEntityDomainStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == zero_tangent
            && check.kind == SketchEntityDomainKind::NonzeroTangentLineSegment2
            && check.status == SketchEntityDomainStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == line3
            && check.kind == SketchEntityDomainKind::NonzeroLengthLineSegment3
            && check.status == SketchEntityDomainStatus::CertifiedValid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == zero_line3
            && check.kind == SketchEntityDomainKind::NonzeroLengthLineSegment3
            && check.status == SketchEntityDomainStatus::CertifiedInvalid
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == full_arc
            && check.kind == SketchEntityDomainKind::NondegenerateArc2
            && check.status == SketchEntityDomainStatus::CertifiedInvalid
    }));
}

#[test]
fn sketch_entity_domains_report_stale_and_wrong_references() {
    let mut sketch = SketchSolveProblem::new();
    let point = sketch.add_point2d("p", real(0), real(0));
    let distance = sketch.add_distance("d", real(1));
    let wrong_unit = point;
    let wrong_radius = point;
    let stale_line = sketch.add_line_segment2("stale tangent", point, SketchEntityHandle(999));
    let circle_with_wrong_radius = sketch.add_circle2("wrong circle", point, point);

    assert!(sketch.add_entity_domain(wrong_unit, SketchEntityDomain::UnitNormal));
    assert!(sketch.add_entity_domain(wrong_radius, SketchEntityDomain::PositiveRadius));
    assert!(sketch.add_entity_domain(stale_line, SketchEntityDomain::NonzeroTangentLineSegment2,));
    assert!(
        sketch.add_entity_domain(circle_with_wrong_radius, SketchEntityDomain::PositiveRadius,)
    );
    assert!(
        !sketch.add_entity_domain(SketchEntityHandle(999), SketchEntityDomain::PositiveRadius,)
    );

    let report = preflight_sketch_entity_domains(&sketch);

    assert_eq!(report.invalid_reference_checks, 4);
    assert!(report.checks.iter().any(|check| {
        check.entity == wrong_unit
            && matches!(
                check.status,
                SketchEntityDomainStatus::WrongEntityKind {
                    handle,
                    expected: "normal"
                } if handle == point
            )
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == stale_line
            && matches!(
                check.status,
                SketchEntityDomainStatus::MissingEntity {
                    handle: SketchEntityHandle(999)
                }
            )
    }));
    assert!(report.checks.iter().any(|check| {
        check.entity == circle_with_wrong_radius
            && matches!(
                check.status,
                SketchEntityDomainStatus::WrongEntityKind {
                    handle,
                    expected: "distance"
                } if handle == point
            )
    }));
    assert_eq!(
        sketch.entities()[distance.0 as usize].domains.len(),
        0,
        "stale-domain attempts must not mutate unrelated entities"
    );
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
    assert!(certificate.entity_domains.all_certified_valid());
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
    let bad_normal = invalid.add_normal2d("bad normal", real(2), real(0));
    assert!(invalid.add_entity_domain(bad_normal, SketchEntityDomain::UnitNormal));
    let invalid_certificate = certify_sketch_construction(&invalid);
    assert_eq!(
        invalid_certificate.status,
        SketchConstructionCertificateStatus::InvalidPreflight
    );
    assert!(invalid_certificate.degeneracies.has_certified_degeneracy());
    assert!(
        invalid_certificate
            .entity_domains
            .has_certified_invalid_domain()
    );

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
            dragged_parameters: Vec::new(),
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
        dragged_parameters: Vec::new(),
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
    assert!(report.preprocessing.requested);
    assert!(report.preprocessing.completed);
    assert_eq!(report.preprocessing.affine_seed_assignments, 1);
    assert!(report.linear_reports.is_empty());
}

#[test]
fn modified_newton_preprocessing_reports_substitution_and_soluble_alone_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let z = Expr::symbol(SymbolId(2), "z");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_variable("z", real(0));
    problem.add_constraint(Constraint::equality(
        "substitute x from y",
        x.clone() - y.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality("solve y alone", y - Expr::int(2)));
    problem.add_constraint(Constraint::equality(
        "quadratic soluble alone",
        z.clone() * z - Expr::int(9),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert!(report.preprocessing.requested);
    assert!(report.preprocessing.completed);
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
    assert_eq!(report.preprocessing.affine_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 1);
    assert_eq!(report.preprocessing.rejected_affine_seed_assignments, 0);
    assert_eq!(report.preprocessing.quadratic_seed_assignments, 0);
    assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 1);
    assert_eq!(report.preprocessing.dragged_parameter_weights, 0);
    assert_eq!(report.preprocessing.invalid_dragged_parameter_weights, 0);
}

#[test]
fn modified_newton_substitution_classes_seed_from_affine_anchor() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "substitute x from y",
        x - y.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality("solve y alone", y - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 1,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 1);
    assert_eq!(report.preprocessing.substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
    assert!(report.linear_reports.is_empty());
}

#[test]
fn modified_newton_substitution_seed_rejects_out_of_bounds_class() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    let x_id = problem.add_variable("x", real(0));
    problem.variables[x_id.0 as usize].upper = Some(real(4));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "substitute x from y",
        x - y.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality("solve y alone", y - Expr::int(2)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 1);
    assert_eq!(report.preprocessing.substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 0);
}

#[test]
fn modified_newton_unanchored_substitution_seed_uses_exact_bound_intersection() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    let x_id = problem.add_variable("x", real(100));
    problem.variables[x_id.0 as usize].upper = Some(real(4));
    let y_id = problem.add_variable("y", real(100));
    problem.variables[y_id.0 as usize].lower = Some(real(0));
    problem.add_constraint(Constraint::equality(
        "substitute x from y",
        x - y - Expr::int(3),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 1,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 0);
    assert_eq!(report.preprocessing.substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
    assert!(report.linear_reports.is_empty());
}

#[test]
fn modified_newton_unanchored_substitution_seed_rejects_empty_bound_intersection() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    let x_id = problem.add_variable("x", real(100));
    problem.variables[x_id.0 as usize].upper = Some(real(2));
    let y_id = problem.add_variable("y", real(100));
    problem.variables[y_id.0 as usize].lower = Some(real(0));
    problem.add_constraint(Constraint::equality(
        "substitute x from y",
        x - y - Expr::int(3),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 0);
}

#[test]
fn modified_newton_affine_soluble_alone_rows_seed_initial_candidate() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("solve x alone", x - Expr::int(7)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 1,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.preprocessing.affine_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 1);
    assert_eq!(report.preprocessing.rejected_affine_seed_assignments, 0);
    assert!(report.linear_reports.is_empty());
}

#[test]
fn modified_newton_affine_seed_rejects_out_of_bounds_assignments() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", real(0));
    problem.variables[variable.0 as usize].upper = Some(real(3));
    problem.add_constraint(Constraint::equality("solve x alone", x - Expr::int(7)));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(report.preprocessing.affine_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 0);
    assert_eq!(report.preprocessing.rejected_affine_seed_assignments, 1);
}

#[test]
fn modified_newton_unique_quadratic_root_seeds_initial_candidate() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "double root",
        x.clone() * x.clone() - Expr::int(6) * x + Expr::int(9),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 1,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
    assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
}

#[test]
fn modified_newton_quadratic_seed_rejects_ambiguous_roots() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "two roots",
        x.clone() * x.clone() - Expr::int(5) * x + Expr::int(6),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.quadratic_seed_assignments, 0);
    assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 1);
}

#[test]
fn modified_newton_quadratic_seed_uses_bounds_to_choose_branch() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", real(0));
    problem.variables[variable.0 as usize].lower = Some(real(3));
    problem.add_constraint(Constraint::equality(
        "bounded two roots",
        x.clone() * x.clone() - Expr::int(5) * x + Expr::int(6),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 1,
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
    assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
}

#[test]
fn modified_newton_quadratic_seed_rejects_bounds_that_do_not_choose_one_branch() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut both_in_bounds = Problem::default();
    let both_variable = both_in_bounds.add_variable("x", real(0));
    both_in_bounds.variables[both_variable.0 as usize].lower = Some(real(2));
    both_in_bounds.variables[both_variable.0 as usize].upper = Some(real(3));
    both_in_bounds.add_constraint(Constraint::equality(
        "both roots retained",
        x.clone() * x.clone() - Expr::int(5) * x.clone() + Expr::int(6),
    ));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        ..SolverConfig::default()
    };

    let both_report = solve_damped_least_squares(SolverState {
        problem: both_in_bounds,
        config: config.clone(),
    });

    assert_eq!(both_report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(both_report.preprocessing.quadratic_seed_assignments, 0);
    assert_eq!(
        both_report
            .preprocessing
            .rejected_quadratic_seed_assignments,
        1
    );

    let mut neither_in_bounds = Problem::default();
    let neither_variable = neither_in_bounds.add_variable("x", real(0));
    neither_in_bounds.variables[neither_variable.0 as usize].upper = Some(real(1));
    neither_in_bounds.add_constraint(Constraint::equality(
        "no retained root",
        x.clone() * x.clone() - Expr::int(5) * x + Expr::int(6),
    ));

    let neither_report = solve_damped_least_squares(SolverState {
        problem: neither_in_bounds,
        config,
    });

    assert_eq!(neither_report.reason, ConvergenceReason::MaxIterations);
    assert_eq!(neither_report.preprocessing.quadratic_seed_assignments, 0);
    assert_eq!(
        neither_report
            .preprocessing
            .rejected_quadratic_seed_assignments,
        1
    );
}

#[test]
fn modified_newton_dragged_parameter_weights_bias_proposal_only() {
    let mut problem = Problem::default();
    let x = problem.add_variable("x", real(0));
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 4,
        damping: real(0),
        dragged_parameters: vec![DraggedParameterWeight {
            variable: x,
            target: real(10),
            weight: real(1),
        }],
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert!(report.preprocessing.requested);
    assert_eq!(report.preprocessing.dragged_parameter_weights, 1);
    assert_eq!(report.preprocessing.invalid_dragged_parameter_weights, 0);
    assert!(!report.linear_reports.is_empty());
}

#[test]
fn modified_newton_dragged_parameter_weights_report_invalid_inputs() {
    let mut problem = Problem::default();
    let fixed = problem.add_variable("fixed", real(0));
    problem.variables[fixed.0 as usize].fixed = true;
    let config = SolverConfig {
        proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
        max_iterations: 0,
        dragged_parameters: vec![
            DraggedParameterWeight {
                variable: fixed,
                target: real(1),
                weight: real(1),
            },
            DraggedParameterWeight {
                variable: hypersolve::VariableId(999),
                target: real(1),
                weight: real(1),
            },
            DraggedParameterWeight {
                variable: fixed,
                target: real(1),
                weight: real(0),
            },
        ],
        ..SolverConfig::default()
    };

    let report = solve_damped_least_squares(SolverState { problem, config });

    assert_eq!(report.reason, ConvergenceReason::MaxIterations);
    assert!(report.preprocessing.requested);
    assert_eq!(report.preprocessing.dragged_parameter_weights, 0);
    assert_eq!(report.preprocessing.invalid_dragged_parameter_weights, 3);
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
fn failed_constraint_diagnostics_classify_contradictions_violations_and_redundancy() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut failed = Problem::default();
    failed.add_variable("x", real(0));
    failed.add_constraint(Constraint::equality("constant contradiction", Expr::int(1)));
    failed.add_constraint(Constraint::equality(
        "candidate miss",
        x.clone() - Expr::int(1),
    ));

    let failed_report = diagnose_failed_constraints(
        &PreparedProblem::new(&failed),
        &context_from_problem(&failed),
    );

    assert!(failed_report.has_blocking_rows());
    assert_eq!(failed_report.blocking_rows, 2);
    assert_eq!(failed_report.certified_contradictions, 1);
    assert_eq!(failed_report.certified_candidate_violations, 1);
    assert!(failed_report.rows.iter().any(|row| {
        row.name == "constant contradiction"
            && row.status == FailedConstraintStatus::CertifiedContradiction
    }));
    assert!(failed_report.rows.iter().any(|row| {
        row.name == "candidate miss"
            && row.status == FailedConstraintStatus::CertifiedCandidateViolation
    }));

    let mut redundant = Problem::default();
    let rx = Expr::symbol(SymbolId(0), "x");
    redundant.add_variable("x", real(2));
    redundant.add_constraint(Constraint::equality(
        "x equals two",
        rx.clone() - Expr::int(2),
    ));
    redundant.add_constraint(Constraint::equality(
        "twice x equals four",
        rx * Expr::int(2) - Expr::int(4),
    ));

    let redundant_report = diagnose_failed_constraints(
        &PreparedProblem::new(&redundant),
        &context_from_problem(&redundant),
    );

    assert!(!redundant_report.has_blocking_rows());
    assert!(redundant_report.only_rank_redundancy());
    assert_eq!(redundant_report.rank_redundant_rows, 2);
    assert!(redundant_report.rows.iter().all(|row| {
        row.status == FailedConstraintStatus::RankRedundant && row.rank_without_row.is_some()
    }));
}

#[test]
fn failed_constraint_diagnostics_preserve_lossy_proposal_only_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(2));
    problem.add_constraint(Constraint::equality("x squared", x.powi(2) - Expr::int(4)));
    let prepared = PreparedProblem::new(&problem);
    let proposal_only = report_lossy_adapter_only_candidate(
        &prepared,
        hypersolve::ProposalEngineReport {
            requested: ProposalEngineKind::ModifiedNewtonLeastSquares,
            used: Some(ProposalEngineKind::ModifiedNewtonLeastSquares),
            precision: ProposalEnginePrecision::LossyF64,
            supported: true,
        },
    );

    let diagnostics = diagnose_failed_constraints_from_certification(&prepared, proposal_only, -64);

    assert_eq!(diagnostics.blocking_rows, 1);
    assert_eq!(diagnostics.did_not_converge_only_rows, 1);
    assert_eq!(
        diagnostics.rows[0].status,
        FailedConstraintStatus::DidNotConvergeOnly
    );
}

#[test]
fn failed_constraint_single_removal_search_reports_clearing_and_still_blocking_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut single = Problem::default();
    single.add_variable("x", real(0));
    single.add_constraint(Constraint::equality(
        "single candidate miss",
        x.clone() - Expr::int(1),
    ));

    let single_search = search_failed_constraint_single_removals(
        &PreparedProblem::new(&single),
        &context_from_problem(&single),
    );

    assert!(single_search.has_single_removal_resolution());
    assert_eq!(single_search.original.blocking_rows, 1);
    assert_eq!(single_search.probes.len(), 1);
    assert_eq!(single_search.clearing_single_removals, 1);
    assert_eq!(
        single_search.probes[0].removal_status,
        FailedConstraintRemovalStatus::ClearsAllBlockingRows
    );

    let y = Expr::symbol(SymbolId(0), "y");
    let mut paired = Problem::default();
    paired.add_variable("y", real(0));
    paired.add_constraint(Constraint::equality(
        "first candidate miss",
        y.clone() - Expr::int(1),
    ));
    paired.add_constraint(Constraint::equality(
        "second candidate miss",
        y - Expr::int(2),
    ));

    let paired_search = search_failed_constraint_single_removals(
        &PreparedProblem::new(&paired),
        &context_from_problem(&paired),
    );

    assert!(!paired_search.has_single_removal_resolution());
    assert_eq!(paired_search.original.blocking_rows, 2);
    assert_eq!(paired_search.probes.len(), 2);
    assert!(paired_search.probes.iter().all(|probe| {
        probe.original_status == FailedConstraintStatus::CertifiedCandidateViolation
            && probe.removal_status
                == FailedConstraintRemovalStatus::StillBlocking { blocking_rows: 1 }
    }));
}

#[test]
fn failed_constraint_pair_removal_search_reports_bounded_two_row_resolutions() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut paired = Problem::default();
    paired.add_variable("x", real(0));
    paired.add_constraint(Constraint::equality(
        "first candidate miss",
        x.clone() - Expr::int(1),
    ));
    paired.add_constraint(Constraint::equality(
        "second candidate miss",
        x.clone() - Expr::int(2),
    ));

    let paired_search = search_failed_constraint_pair_removals(
        &PreparedProblem::new(&paired),
        &context_from_problem(&paired),
    );

    assert!(paired_search.has_pair_removal_resolution());
    assert_eq!(paired_search.original.blocking_rows, 2);
    assert_eq!(paired_search.probes.len(), 1);
    assert_eq!(paired_search.clearing_pair_removals, 1);
    assert_eq!(
        paired_search.probes[0].removal_status,
        FailedConstraintRemovalStatus::ClearsAllBlockingRows
    );

    let y = Expr::symbol(SymbolId(0), "y");
    let mut triple = Problem::default();
    triple.add_variable("y", real(0));
    triple.add_constraint(Constraint::equality(
        "first triple miss",
        y.clone() - Expr::int(1),
    ));
    triple.add_constraint(Constraint::equality(
        "second triple miss",
        y.clone() - Expr::int(2),
    ));
    triple.add_constraint(Constraint::equality("third triple miss", y - Expr::int(3)));

    let triple_search = search_failed_constraint_pair_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
    );

    assert!(!triple_search.has_pair_removal_resolution());
    assert_eq!(triple_search.original.blocking_rows, 3);
    assert_eq!(triple_search.probes.len(), 3);
    assert!(triple_search.probes.iter().all(|probe| {
        probe.removal_status == FailedConstraintRemovalStatus::StillBlocking { blocking_rows: 1 }
    }));
}

#[test]
fn failed_constraint_set_removal_search_reports_bounded_cardinality_resolutions() {
    let y = Expr::symbol(SymbolId(0), "y");
    let mut triple = Problem::default();
    triple.add_variable("y", real(0));
    triple.add_constraint(Constraint::equality(
        "first triple miss",
        y.clone() - Expr::int(1),
    ));
    triple.add_constraint(Constraint::equality(
        "second triple miss",
        y.clone() - Expr::int(2),
    ));
    triple.add_constraint(Constraint::equality("third triple miss", y - Expr::int(3)));

    let zero_search = search_failed_constraint_set_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
        0,
    );
    assert_eq!(zero_search.original.blocking_rows, 3);
    assert!(zero_search.probes.is_empty());
    assert!(!zero_search.has_removal_resolution());

    let pair_bounded_search = search_failed_constraint_set_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
        2,
    );
    assert_eq!(pair_bounded_search.original.blocking_rows, 3);
    assert_eq!(pair_bounded_search.probes.len(), 6);
    assert_eq!(pair_bounded_search.clearing_removals, 0);
    assert!(!pair_bounded_search.has_removal_resolution());

    let triple_search = search_failed_constraint_set_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
        3,
    );
    assert_eq!(triple_search.original.blocking_rows, 3);
    assert_eq!(triple_search.max_cardinality, 3);
    assert_eq!(triple_search.probes.len(), 7);
    assert_eq!(triple_search.clearing_removals, 1);
    assert!(triple_search.has_removal_resolution());
    assert_eq!(
        triple_search
            .probes
            .last()
            .expect("triple combination should be probed")
            .removal_status,
        FailedConstraintRemovalStatus::ClearsAllBlockingRows
    );
}

#[test]
fn failed_constraint_minimal_removal_search_reports_first_clearing_cardinality() {
    let y = Expr::symbol(SymbolId(0), "y");
    let mut triple = Problem::default();
    triple.add_variable("y", real(0));
    triple.add_constraint(Constraint::equality(
        "first triple miss",
        y.clone() - Expr::int(1),
    ));
    triple.add_constraint(Constraint::equality(
        "second triple miss",
        y.clone() - Expr::int(2),
    ));
    triple.add_constraint(Constraint::equality("third triple miss", y - Expr::int(3)));

    let pair_bound = search_failed_constraint_minimal_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
        2,
    );
    assert_eq!(pair_bound.original.blocking_rows, 3);
    assert_eq!(pair_bound.max_cardinality, 2);
    assert_eq!(pair_bound.minimal_cardinality, None);
    assert_eq!(pair_bound.probes.len(), 6);
    assert_eq!(pair_bound.clearing_removals, 0);
    assert!(!pair_bound.has_minimal_removal_resolution());

    let triple_bound = search_failed_constraint_minimal_removals(
        &PreparedProblem::new(&triple),
        &context_from_problem(&triple),
        3,
    );
    assert_eq!(triple_bound.original.blocking_rows, 3);
    assert_eq!(triple_bound.minimal_cardinality, Some(3));
    assert_eq!(triple_bound.probes.len(), 1);
    assert_eq!(triple_bound.clearing_removals, 1);
    assert!(triple_bound.has_minimal_removal_resolution());
    assert_eq!(triple_bound.probes[0].constraint_indices, vec![0, 1, 2]);
    assert_eq!(
        triple_bound.probes[0].removal_status,
        FailedConstraintRemovalStatus::ClearsAllBlockingRows
    );
}

#[test]
fn sketch_failed_constraint_report_maps_exact_rows_to_retained_constraints() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(3), real(4));
    let too_short = sketch.add_distance("too short", real(4));
    let valid_distance = sketch.add_distance("valid distance", real(5));
    let failed = sketch.add_point_point_distance("failed sketch distance", a, b, too_short);
    let valid = sketch.add_point_point_distance("valid sketch distance", a, b, valid_distance);
    let stale = sketch.add_point_point_distance(
        "stale sketch distance",
        a,
        SketchEntityHandle(999),
        valid_distance,
    );

    let report = diagnose_sketch_failed_constraints(&sketch);

    assert!(report.has_blocking_rows());
    assert_eq!(report.lowering_failure_rows, 1);
    assert_eq!(report.exact_failure_rows, 1);
    assert_eq!(report.rank_redundant_rows, 0);
    assert_eq!(report.blocking_rows, 2);
    assert!(report.rows.iter().any(|row| {
        row.constraint == failed
            && row.constraint_name == "failed sketch distance"
            && matches!(
                row.status,
                SketchFailedConstraintStatus::ExactFailure(
                    FailedConstraintStatus::CertifiedCandidateViolation
                )
            )
            && row
                .failed
                .as_ref()
                .is_some_and(|failed| failed.name == "failed sketch distance")
    }));
    assert!(report.rows.iter().any(|row| {
        row.constraint == stale
            && matches!(
                row.status,
                SketchFailedConstraintStatus::LoweringFailure(
                    SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999))
                )
            )
            && row.failed.is_none()
    }));
    assert!(!report.rows.iter().any(|row| row.constraint == valid));
}

#[test]
fn sketch_failed_constraint_report_maps_affine_rank_redundancy_to_sources() {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", real(0), real(0));
    let b = sketch.add_point2d("b", real(0), real(0));
    let redundant_a = sketch.add_points_coincident("redundant coincidence a", a, b);
    let redundant_b = sketch.add_points_coincident("redundant coincidence b", a, b);

    let report = diagnose_sketch_failed_constraints(&sketch);

    assert!(!report.has_blocking_rows());
    assert!(report.only_rank_redundancy());
    assert_eq!(report.blocking_rows, 0);
    assert_eq!(report.lowering_failure_rows, 0);
    assert_eq!(report.exact_failure_rows, 0);
    assert_eq!(report.rank_redundant_rows, 4);
    assert_eq!(report.rows.len(), 4);
    assert!(report.rows.iter().all(|row| {
        row.status == SketchFailedConstraintStatus::RankRedundant
            && row
                .failed
                .as_ref()
                .is_some_and(|failed| failed.rank_without_row.is_some())
    }));
    assert_eq!(
        report
            .rows
            .iter()
            .filter(|row| row.constraint == redundant_a)
            .count(),
        2
    );
    assert_eq!(
        report
            .rows
            .iter()
            .filter(|row| row.constraint == redundant_b)
            .count(),
        2
    );
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
fn candidate_batch_predicate_schedule_chunks_active_rows_deterministically() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("active 0", x.clone()));
    let mut inactive = Constraint::equality("inactive", x.clone());
    inactive.active = false;
    problem.add_constraint(inactive);
    problem.add_constraint(Constraint::equality("active 2", x.clone()));
    problem.add_constraint(Constraint::equality("active 3", x));
    let prepared = PreparedProblem::new(&problem);

    let schedule = schedule_candidate_batch_predicates(
        &prepared,
        2,
        BatchPredicateScheduleConfig {
            max_rows_per_work_item: 2,
        },
    )
    .unwrap();

    assert_eq!(schedule.candidate_count, 2);
    assert_eq!(schedule.active_row_count, 3);
    assert_eq!(schedule.work_item_count(), 4);
    assert_eq!(schedule.work_items[0].source_constraints, vec![0, 2]);
    assert_eq!(schedule.work_items[1].source_constraints, vec![3]);
    assert_eq!(schedule.work_items[2].candidate_index, 1);
    assert_eq!(schedule.work_items[2].source_constraints, vec![0, 2]);
    assert_eq!(
        schedule_candidate_batch_predicates(
            &prepared,
            1,
            BatchPredicateScheduleConfig {
                max_rows_per_work_item: 0,
            },
        )
        .unwrap_err(),
        BatchPredicateScheduleError::ZeroRowsPerWorkItem
    );
}

#[test]
fn prepared_sparse_linear_batch_replay_is_public_and_deterministic() {
    let terms = vec![
        SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: real(1),
        },
        SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: real(1),
        },
        SparseResidualTerm {
            row: 1,
            column: 1,
            coefficient: real(3),
        },
    ];
    let rhs = vec![real(4), real(9)];
    let prepared = prepare_sparse_linear_residual_system(2, 2, &terms, &rhs).unwrap();

    assert_eq!(prepared.row_terms()[0], vec![(0, real(2))]);

    let report = prepared
        .replay_batch(&[vec![real(2), real(3)], vec![real(1), real(3)]], -64)
        .unwrap();
    let wrapper_report = replay_sparse_linear_residual_batch(
        2,
        2,
        &terms,
        &rhs,
        &[vec![real(2), real(3)], vec![real(1), real(3)]],
        -64,
    )
    .unwrap();

    assert_eq!(report, wrapper_report);
    assert_eq!(report.candidate_count, 2);
    assert_eq!(report.accepted_candidates, 1);
    assert_eq!(report.rejected_candidates, 1);
    assert_eq!(report.candidates[0].candidate_index, 0);
    assert_eq!(
        report.candidates[0].status,
        SparseResidualBatchStatus::Accepted
    );
    assert_eq!(report.candidates[1].candidate_index, 1);
    assert_eq!(report.candidates[1].first_nonzero_row, Some(0));
    assert_eq!(report.candidates[1].nonzero_rows, vec![0]);
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
fn affine_active_set_regeneration_solves_masked_rows_and_audits_source_problem() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "sum",
        x.clone() + y.clone() - Expr::int(7),
    ));
    problem.add_constraint(Constraint::equality(
        "difference",
        x.clone() - y.clone() - Expr::int(1),
    ));
    let mut inactive_bound = Constraint::equality("x upper", x - Expr::int(5));
    inactive_bound.kind = ConstraintKind::LessOrEqual;
    inactive_bound.active = false;
    problem.add_constraint(inactive_bound);

    let report = regenerate_active_set_affine_candidate(
        &PreparedProblem::new(&problem),
        &[true, true, false],
        CandidateCertificationConfig::default(),
    );

    assert_eq!(report.status, ActiveSetAffineRegenerationStatus::Certified);
    let candidate = report.candidate.as_ref().unwrap();
    assert_eq!(candidate.bindings().get(&SymbolId(0)), Some(&real(4)));
    assert_eq!(candidate.bindings().get(&SymbolId(1)), Some(&real(3)));
    assert!(report.audit.as_ref().unwrap().all_consistent());
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
