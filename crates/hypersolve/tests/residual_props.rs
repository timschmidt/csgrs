use hyperreal::{Real, RealSign};
use hypersolve::{
    BatchCandidateStatus, CertifiedCandidateStatus, Constraint, ConvergenceReason, DomainCheckKind,
    DomainCheckStatus, DraggedParameterWeight, EqualitySubstitution, EqualitySubstitutionProblem,
    Expr, FailedConstraintRemovalStatus, FailedConstraintStatus, IntervalBoxCertificationPackage,
    IntervalBoxCertificationStatus, MultivariateQuadraticKrawczykStatus, PreparedProblem,
    PreparedSolverBlock, Problem, ProposalEngineKind, ProposalEnginePrecision,
    ProposalEngineReport, RootIsolationStatus, SketchArcEndpoint, SketchArcLengthSweep,
    SketchArcPointSweep, SketchArcTangencyBranch, SketchCircleTangencyBranch,
    SketchConstructionCertificateStatus, SketchDegeneracyKind, SketchDegeneracyStatus,
    SketchEntityDomain, SketchEntityDomainKind, SketchEntityDomainStatus,
    SketchFailedConstraintStatus, SketchGeneratedRowStatus, SketchLineEndpoint,
    SketchParameterDomain, SketchParameterDomainStatus, SketchResidualFormKind,
    SketchResidualFormRole, SketchResidualFormsStatus, SketchResidualStrategy,
    SketchRoundTripMetadata, SketchSolveProblem, SketchTangentOrientation,
    SketchUnitToleranceStatus, SketchWorkplaneFrameStatus, SolverBlockRowKind, SolverConfig,
    SolverPoint2, SolverState, SymbolId, VariableBall, apply_equality_substitution_classes,
    audit_sketch_unit_tolerances, certify_affine_krawczyk_box, certify_candidate,
    certify_candidate_batch, certify_candidate_domains, certify_direct_univariate_quadratic_roots,
    certify_interval_box_candidate, certify_multivariate_quadratic_interval_candidate,
    certify_multivariate_quadratic_krawczyk_box, certify_quadratic_interval_candidate,
    certify_sketch_construction, certify_univariate_quadratic_alpha,
    certify_univariate_quadratic_krawczyk_box, context_from_problem, determinant_bareiss,
    diagnose_failed_constraints, diagnose_sketch_failed_constraints,
    eliminate_affine_rows_with_substitution_classes,
    enumerate_direct_univariate_quadratic_branches, isolate_univariate_polynomial_roots,
    lift_sketch_point2_to_workplane3, preflight_sketch_degeneracies,
    preflight_sketch_entity_domains, preflight_sketch_parameter_domains,
    report_lossy_adapter_only_candidate, resultant_univariate_polynomials,
    search_failed_constraint_minimal_removals, search_failed_constraint_pair_removals,
    search_failed_constraint_set_removals, search_failed_constraint_single_removals,
    sketch_range_builders, solve_damped_least_squares, solve_dense_linear_system_bareiss,
    solve_direct_affine_system, solve_direct_univariate_quadratic_equalities,
    squared_distance_equation, subresultant_chain_univariate_polynomials,
    validate_equality_substitutions,
};
use proptest::prelude::*;

fn expected_sign(value: i64) -> RealSign {
    match value.cmp(&0) {
        std::cmp::Ordering::Less => RealSign::Negative,
        std::cmp::Ordering::Equal => RealSign::Zero,
        std::cmp::Ordering::Greater => RealSign::Positive,
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn prepared_problem_constant_residual_sign_counts_are_exact(
        constants in prop::collection::vec(-32_i16..=32, 1..24),
    ) {
        let mut problem = Problem::default();
        problem.add_variable("x", hyperreal::Real::from(0));
        for (index, value) in constants.iter().enumerate() {
            problem.add_constraint(Constraint::equality(
                format!("constant {index}"),
                Expr::int(i64::from(*value)),
            ));
        }

        let prepared = PreparedProblem::new(&problem);
        let facts = prepared.facts();
        let expected_zero = constants.iter().filter(|value| **value == 0).count();
        let expected_nonzero = constants.len() - expected_zero;

        prop_assert_eq!(facts.active_constraint_count, constants.len());
        prop_assert_eq!(facts.prepared_affine_active_rows, constants.len());
        prop_assert!(facts.all_active_rows_prepared_affine());
        prop_assert_eq!(facts.known_zero_constant_active_rows, expected_zero);
        prop_assert_eq!(facts.known_nonzero_constant_active_rows, expected_nonzero);
        prop_assert_eq!(facts.unknown_sign_constant_active_rows, 0);
        prop_assert_eq!(
            facts.has_known_nonzero_constant_residual(),
            expected_nonzero > 0
        );

        for (row, value) in prepared.constraints().iter().zip(constants) {
            prop_assert!(row.is_constant_row());
            prop_assert_eq!(row.residual_constant_sign, Some(expected_sign(i64::from(value))));
            prop_assert_eq!(row.is_known_zero_constant_row(), value == 0);
            prop_assert_eq!(row.is_known_nonzero_constant_row(), value != 0);
        }
    }

    #[test]
    fn prepared_problem_variable_rows_do_not_get_constant_residual_signs(
        coefficients in prop::collection::vec(-16_i16..=16, 1..24),
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", hyperreal::Real::from(0));
        for (index, coefficient) in coefficients.iter().enumerate() {
            problem.add_constraint(Constraint::equality(
                format!("row {index}"),
                x.clone() * Expr::int(i64::from(*coefficient)) + Expr::int(1),
            ));
        }

        let prepared = PreparedProblem::new(&problem);

        prop_assert_eq!(prepared.facts().active_constraint_count, coefficients.len());
        prop_assert_eq!(prepared.facts().prepared_affine_active_rows, coefficients.len());
        prop_assert!(prepared.facts().all_active_rows_prepared_affine());
        prop_assert_eq!(prepared.facts().known_zero_constant_active_rows, 0);
        prop_assert_eq!(prepared.facts().known_nonzero_constant_active_rows, 0);
        prop_assert_eq!(prepared.facts().unknown_sign_constant_active_rows, 0);
        for row in prepared.constraints() {
            prop_assert!(!row.is_constant_row());
            prop_assert_eq!(row.residual_constant_sign, None);
        }
    }

    #[test]
    fn prepared_affine_residuals_match_expression_evaluation(
        a in -16_i16..=16,
        b in -16_i16..=16,
        c in -64_i16..=64,
        x_value in -16_i16..=16,
        y_value in -16_i16..=16,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", hyperreal::Real::from(i64::from(x_value)));
        problem.add_variable("y", hyperreal::Real::from(i64::from(y_value)));
        problem.add_constraint(Constraint::equality(
            "generated affine",
            x * Expr::int(i64::from(a)) + y * Expr::int(i64::from(b)) + Expr::int(i64::from(c)),
        ));
        let context = context_from_problem(&problem);
        let prepared = PreparedProblem::new(&problem);
        let affine = prepared.affine_residuals()[0]
            .as_ref()
            .expect("generated affine row should prepare");

        prop_assert_eq!(affine.coefficients()[0].clone(), hyperreal::Real::from(i64::from(a)));
        prop_assert_eq!(affine.coefficients()[1].clone(), hyperreal::Real::from(i64::from(b)));
        prop_assert_eq!(affine.constant().clone(), hyperreal::Real::from(i64::from(c)));
        prop_assert_eq!(
            affine.eval_real(&problem.variables, context.bindings()).unwrap(),
            problem.constraints[0].residual.eval_real(context.bindings()).unwrap()
        );
        prop_assert_eq!(
            prepared.evaluate_residuals(&context).unwrap()[0].value.clone(),
            problem.constraints[0].residual.eval_real(context.bindings()).unwrap()
        );
    }

    #[test]
    fn candidate_certification_matches_generated_affine_zero_status(
        a in -16_i16..=16,
        x_value in -16_i16..=16,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", hyperreal::Real::from(i64::from(x_value)));
        problem.add_constraint(Constraint::equality(
            "generated affine candidate",
            x * Expr::int(i64::from(a)) - Expr::int(i64::from(a) * i64::from(x_value)),
        ));
        let prepared = PreparedProblem::new(&problem);
        let report = certify_candidate(&prepared, &context_from_problem(&problem));

        prop_assert_eq!(report.rows.len(), 1);
        prop_assert_eq!(matches!(
            report.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ), true);
        prop_assert!(report.all_satisfied());
    }

    #[test]
    fn candidate_batch_generated_affine_targets_preserve_input_order_and_status(
        target in -16_i16..=16,
        values in prop::collection::vec(-16_i16..=16, 1..16),
    ) {
        let target = i64::from(target);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality("generated batch target", x - Expr::int(target)));
        let prepared = PreparedProblem::new(&problem);
        let candidates = values
            .iter()
            .map(|value| {
                let mut context = context_from_problem(&problem);
                context.bind(SymbolId(0), Real::from(i64::from(*value)));
                context
            })
            .collect::<Vec<_>>();

        let report = certify_candidate_batch(&prepared, &candidates);

        prop_assert_eq!(report.candidate_count, values.len());
        prop_assert_eq!(
            report.certified_candidates,
            values.iter().filter(|value| i64::from(**value) == target).count()
        );
        for (index, replay) in report.candidates.iter().enumerate() {
            prop_assert_eq!(replay.candidate_index, index);
            let expected = if i64::from(values[index]) == target {
                BatchCandidateStatus::Certified
            } else {
                BatchCandidateStatus::Rejected
            };
            prop_assert_eq!(replay.status, expected);
        }
    }

    #[test]
    fn sketch_point_distance_rows_replay_generated_integer_triangles(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        dx in -8_i16..=8,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", hyperreal::Real::from(ax), hyperreal::Real::from(ay));
        let b = sketch.add_point2d(
            "b",
            hyperreal::Real::from(ax + dx),
            hyperreal::Real::from(ay),
        );
        let d = sketch.add_distance("d", hyperreal::Real::from(dx.abs()));
        sketch.add_point_point_distance("distance", a, b, d);

        let lowered = sketch.lower_to_problem();

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            &lowered.rows[0].status,
            &SketchGeneratedRowStatus::Generated
        );
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::SquaredDistance));
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let certified_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(certified_zero);
    }

    #[test]
    fn sketch_point_distance_range_rows_replay_generated_integer_segments(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        dx in -8_i16..=8,
        lower_slack in 0_i16..=8,
        upper_slack in 0_i16..=8,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let distance = dx.abs();
        let lower = (distance - i64::from(lower_slack)).max(0);
        let upper = distance + i64::from(upper_slack);
        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
        let b = sketch.add_point2d("b", Real::from(ax + dx), Real::from(ay));
        sketch.add_point_point_distance_range(
            "clearance window",
            a,
            b,
            Some(Real::from(lower)),
            Some(Real::from(upper)),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_distance_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::BoundedSquaredDistance)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_distance_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_equal_length_rows_match_generated_integer_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        bx in -8_i16..=8,
        by in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let b0 = sketch.add_point2d("b0", Real::from(bx), Real::from(by));
        let b1 = sketch.add_point2d("b1", Real::from(bx - dy), Real::from(by + dx));
        let line_a = sketch.add_line_segment2("a", a0, a1);
        let line_b = sketch.add_line_segment2("b", b0, b1);
        sketch.add_equal_length_lines2("equal length", line_a, line_b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredLineLengthEquality)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_equal_radius_rows_match_generated_integer_radii(
        radius in 0_i16..=16,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
    ) {
        let radius = i64::from(radius);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let mut sketch = SketchSolveProblem::new();
        let c0 = sketch.add_point2d("c0", Real::from(0), Real::from(0));
        let c1 = sketch.add_point2d("c1", Real::from(cx), Real::from(cy));
        let r0 = sketch.add_distance("r0", Real::from(radius));
        let r1 = sketch.add_distance("r1", Real::from(radius));
        let circle = sketch.add_circle2("circle", c0, r0);
        let arc = sketch.add_arc_of_circle2("arc", c1, c0, c1, r1);
        sketch.add_equal_radius2("equal radius", circle, arc);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::RadiusEquality)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_concentric_rows_match_generated_circle_arc_centers(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=16,
        dx in -6_i16..=6,
        dy in -6_i16..=6,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let circle = sketch.add_circle2("circle", center, radius_entity);
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let handle = sketch.add_concentric2("concentric", circle, arc);
        if dx != 0 || dy != 0 {
            let shifted_center =
                sketch.add_point2d("shifted center", Real::from(cx + dx), Real::from(cy + dy));
            let shifted = sketch.add_circle2("shifted", shifted_center, radius_entity);
            sketch.add_concentric2("bad concentric", circle, shifted);
        }

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::Concentricity));
        prop_assert_eq!(lowered.rows[1].strategy, Some(SketchResidualStrategy::Concentricity));
        let first_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let second_zero = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(first_zero);
        prop_assert!(second_zero);
        if dx != 0 || dy != 0 {
            prop_assert!(!certification.all_satisfied());
        }
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        for form in &forms.forms {
            prop_assert_eq!(
                form.kind,
                SketchResidualFormKind::ConcentricCenterCoordinatePolynomial
            );
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
    }

    #[test]
    fn sketch_length_ratio_rows_match_generated_integer_scaled_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        bx in -8_i16..=8,
        by in -8_i16..=8,
        dx in -6_i16..=6,
        dy in -6_i16..=6,
        scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d(
            "a1",
            Real::from(ax + scale * dx),
            Real::from(ay + scale * dy),
        );
        let b0 = sketch.add_point2d("b0", Real::from(bx), Real::from(by));
        let b1 = sketch.add_point2d("b1", Real::from(bx + dx), Real::from(by + dy));
        let line_a = sketch.add_line_segment2("a", a0, a1);
        let line_b = sketch.add_line_segment2("b", b0, b1);
        sketch.add_length_ratio_lines2(
            "length ratio",
            line_a,
            line_b,
            Real::from(scale),
            Real::from(1),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredLineLengthRatio)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_length_difference_rows_match_generated_horizontal_lengths(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        shorter in 0_i16..=12,
        difference in 0_i16..=12,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let shorter = i64::from(shorter);
        let difference = i64::from(difference);
        let mut sketch = SketchSolveProblem::new();
        let start = sketch.add_point2d("start", Real::from(x0), Real::from(y0));
        let shorter_end = sketch.add_point2d("shorter end", Real::from(x0 + shorter), Real::from(y0));
        let longer_end = sketch.add_point2d(
            "longer end",
            Real::from(x0 + shorter + difference),
            Real::from(y0),
        );
        let shorter_line = sketch.add_line_segment2("shorter", start, shorter_end);
        let longer_line = sketch.add_line_segment2("longer", start, longer_end);
        let difference = sketch.add_distance("difference", Real::from(difference));
        sketch.add_length_difference_lines2(
            "length difference",
            longer_line,
            shorter_line,
            difference,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_difference_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::SquaredLineLengthDifference)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_difference_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_point_line_distance_rows_match_generated_horizontal_offsets(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        px in -8_i16..=8,
        offset in 0_i16..=12,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let px = i64::from(px);
        let offset = i64::from(offset);
        let mut sketch = SketchSolveProblem::new();
        let start = sketch.add_point2d("start", Real::from(x0), Real::from(y0));
        let end = sketch.add_point2d("end", Real::from(x0 + 5), Real::from(y0));
        let point = sketch.add_point2d("point", Real::from(px), Real::from(y0 + offset));
        let line = sketch.add_line_segment2("line", start, end);
        let distance = sketch.add_distance("distance", Real::from(offset));
        sketch.add_point_line_distance2("point line distance", point, line, distance);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredPointLineDistance)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_equal_length_point_line_distance_rows_match_horizontal_offsets(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        length in 0_i16..=12,
        px in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let length = i64::from(length);
        let px = i64::from(px);
        let mut sketch = SketchSolveProblem::new();
        let length_start = sketch.add_point2d("length start", Real::from(0), Real::from(0));
        let length_end = sketch.add_point2d("length end", Real::from(length), Real::from(0));
        let line_start = sketch.add_point2d("line start", Real::from(x0), Real::from(y0));
        let line_end = sketch.add_point2d("line end", Real::from(x0 + 5), Real::from(y0));
        let point = sketch.add_point2d("point", Real::from(px), Real::from(y0 + length));
        let length_line = sketch.add_line_segment2("length line", length_start, length_end);
        let distance_line = sketch.add_line_segment2("distance line", line_start, line_end);
        sketch.add_equal_length_point_line_distance2(
            "length equals point-line",
            length_line,
            point,
            distance_line,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredLineLengthPointLineDistance)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_equal_point_line_distances_rows_match_horizontal_offsets(
        y0 in -8_i16..=8,
        y1 in -8_i16..=8,
        x0 in -8_i16..=8,
        x1 in -8_i16..=8,
        offset in 0_i16..=12,
    ) {
        let y0 = i64::from(y0);
        let y1 = i64::from(y1);
        let x0 = i64::from(x0);
        let x1 = i64::from(x1);
        let offset = i64::from(offset);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(0), Real::from(y0));
        let a1 = sketch.add_point2d("a1", Real::from(5), Real::from(y0));
        let b0 = sketch.add_point2d("b0", Real::from(3), Real::from(y1));
        let b1 = sketch.add_point2d("b1", Real::from(8), Real::from(y1));
        let point_a = sketch.add_point2d("point a", Real::from(x0), Real::from(y0 + offset));
        let point_b = sketch.add_point2d("point b", Real::from(x1), Real::from(y1 + offset));
        let line_a = sketch.add_line_segment2("line a", a0, a1);
        let line_b = sketch.add_line_segment2("line b", b0, b1);
        sketch.add_equal_point_line_distances2(
            "equal point-line distances",
            point_a,
            line_a,
            point_b,
            line_b,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredEqualPointLineDistances)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_line_parallel_rows_match_generated_integer_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
        scale in -4_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let b0 = sketch.add_point2d("b0", Real::from(3), Real::from(-2));
        let b1 = sketch.add_point2d(
            "b1",
            Real::from(3 + scale * dx),
            Real::from(-2 + scale * dy),
        );
        let line_a = sketch.add_line_segment2("a", a0, a1);
        let line_b = sketch.add_line_segment2("b", b0, b1);
        sketch.add_parallel_lines2("parallel", line_a, line_b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::DirectionCrossProduct)
        );
        let certified_parallel = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(certified_parallel);
    }

    #[test]
    fn sketch_line_perpendicular_rows_match_generated_integer_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let b0 = sketch.add_point2d("b0", Real::from(1), Real::from(1));
        let b1 = sketch.add_point2d("b1", Real::from(1 - dy), Real::from(1 + dx));
        let line_a = sketch.add_line_segment2("a", a0, a1);
        let line_b = sketch.add_line_segment2("b", b0, b1);
        sketch.add_perpendicular_lines2("perpendicular", line_a, line_b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::DirectionDotProduct)
        );
        let certified_perpendicular = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(certified_perpendicular);
    }

    #[test]
    fn sketch_same_direction_rows_match_generated_positive_scaled_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
        scale in 1_i16..=5,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let b0 = sketch.add_point2d("b0", Real::from(3), Real::from(-2));
        let b1 = sketch.add_point2d(
            "b1",
            Real::from(3 + scale * dx),
            Real::from(-2 + scale * dy),
        );
        let line_a = sketch.add_line_segment2("a", a0, a1);
        let line_b = sketch.add_line_segment2("b", b0, b1);
        sketch.add_same_direction_lines2("same direction", line_a, line_b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_same_orientation_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::DirectionSameOrientation)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_same_orientation_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_tangent_same_direction_rows_match_generated_positive_scaled_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
        scale in 1_i16..=5,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let b0 = sketch.add_point2d("b0", Real::from(-4), Real::from(7));
        let b1 = sketch.add_point2d(
            "b1",
            Real::from(-4 + scale * dx),
            Real::from(7 + scale * dy),
        );
        let candidate = sketch.add_line_segment2("candidate tangent", a0, a1);
        let target = sketch.add_line_segment2("target tangent", b0, b1);
        sketch.add_tangent_same_direction_lines2("same tangent", candidate, target);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_tangent_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::TangentSameDirection)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_tangent_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_tangent_residual_forms_replay_generated_positive_scaled_directions(
        dx in -8_i16..=8,
        dy in -8_i16..=8,
        scale in 1_i16..=5,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(0), Real::from(0));
        let a1 = sketch.add_point2d("a1", Real::from(dx), Real::from(dy));
        let b0 = sketch.add_point2d("b0", Real::from(3), Real::from(-5));
        let b1 = sketch.add_point2d(
            "b1",
            Real::from(3 + scale * dx),
            Real::from(-5 + scale * dy),
        );
        let candidate = sketch.add_line_segment2("candidate tangent", a0, a1);
        let target = sketch.add_line_segment2("target tangent", b0, b1);
        let handle = sketch.add_tangent_same_direction_lines2("same tangent", candidate, target);

        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&sketch.lower_to_problem().problem);

        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(forms.forms[0].kind, SketchResidualFormKind::TangentCrossProductPredicate);
        prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(forms.forms[1].kind, SketchResidualFormKind::TangentDotProductPredicate);
        prop_assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(
            forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
        let dot = forms.forms[1].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_line_arc_length_rows_replay_generated_half_circles(
        radius in 1_i16..=8,
        x in -4_i16..=4,
        y in -4_i16..=4,
    ) {
        let r = i64::from(radius);
        let x = i64::from(x);
        let y = i64::from(y);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(x), Real::from(y));
        let start = sketch.add_point2d("start", Real::from(x + r), Real::from(y));
        let end = sketch.add_point2d("end", Real::from(x - r), Real::from(y));
        let radius_entity = sketch.add_distance("radius", Real::from(r));
        let arc = sketch.add_arc_of_circle2("half arc", center, start, end, radius_entity);
        let line_start = sketch.add_point2d("line start", Real::from(0), Real::from(0));
        let line_end = sketch.add_point2d("line end", Real::from(r) * Real::pi(), Real::from(0));
        let line = sketch.add_line_segment2("matching line", line_start, line_end);
        let handle = sketch.add_equal_line_arc_length2("line arc length", line, arc);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 3);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::LineArcLength)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[2].kind,
            SketchResidualFormKind::LineArcLengthTranscendentalEquality
        );
        prop_assert_eq!(forms.forms[2].role, SketchResidualFormRole::ExactProof);
    }

    #[test]
    fn sketch_line_arc_sweep_length_rows_replay_generated_quarter_major_sweeps(
        radius in 1_i16..=8,
        x in -4_i16..=4,
        y in -4_i16..=4,
    ) {
        let r = i64::from(radius);
        let x = i64::from(x);
        let y = i64::from(y);
        let three_halves = Real::new(hyperreal::Rational::fraction(3, 2).unwrap());
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(x), Real::from(y));
        let start = sketch.add_point2d("start", Real::from(x + r), Real::from(y));
        let end = sketch.add_point2d("end", Real::from(x), Real::from(y + r));
        let radius_entity = sketch.add_distance("radius", Real::from(r));
        let arc = sketch.add_arc_of_circle2("quarter arc", center, start, end, radius_entity);
        let line_start = sketch.add_point2d("line start", Real::from(0), Real::from(0));
        let line_end = sketch.add_point2d(
            "line end",
            Real::from(r) * Real::pi() * three_halves,
            Real::from(0),
        );
        let line = sketch.add_line_segment2("major line", line_start, line_end);
        let handle = sketch.add_equal_line_arc_sweep_length2(
            "clockwise major sweep",
            line,
            arc,
            SketchArcLengthSweep::ClockwiseMajor,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 4);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::LineArcSweepLength)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 4);
        prop_assert_eq!(
            forms.forms[2].kind,
            SketchResidualFormKind::LineArcSweepLengthBranchPredicate
        );
        prop_assert_eq!(
            forms.forms[3].kind,
            SketchResidualFormKind::LineArcSweepLengthTranscendentalEquality
        );
    }

    #[test]
    fn sketch_projected_line_arc_sweep_length_rows_replay_identity_workplane_major_sweeps(
        radius in 1_i16..=8,
        x in -4_i16..=4,
        y in -4_i16..=4,
        z in -4_i16..=4,
    ) {
        let r = i64::from(radius);
        let x = i64::from(x);
        let y = i64::from(y);
        let z = i64::from(z);
        let three_halves = Real::new(hyperreal::Rational::fraction(3, 2).unwrap());
        let mut sketch = SketchSolveProblem::new();
        let origin3 = sketch.add_point3d("origin3", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin3, normal);
        let center = sketch.add_point2d("center", Real::from(x), Real::from(y));
        let start = sketch.add_point2d("start", Real::from(x + r), Real::from(y));
        let end = sketch.add_point2d("end", Real::from(x), Real::from(y + r));
        let radius_entity = sketch.add_distance("radius", Real::from(r));
        let arc = sketch.add_arc_of_circle2("quarter arc", center, start, end, radius_entity);
        let line_start = sketch.add_point3d("line start", Real::from(0), Real::from(0), Real::from(z));
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(r) * Real::pi() * three_halves,
            Real::from(0),
            Real::from(z + 7),
        );
        let line = sketch.add_line_segment3("projected major line", line_start, line_end);
        let handle = sketch.add_projected_equal_line_arc_sweep_length3(
            "projected sweep length",
            workplane,
            line,
            arc,
            SketchArcLengthSweep::ClockwiseMajor,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 5);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_arc_rows_generated = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedLineArcSweepLength)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_arc_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ProjectedLineArcSweepLengthTranscendentalEquality
        );
    }

    #[test]
    fn sketch_arc_line_tangent_rows_match_generated_integer_circle_tangents(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
        tangent_length in 1_i16..=12,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let tangent_length = i64::from(tangent_length);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let line_end = sketch.add_point2d(
            "line end",
            Real::from(cx + radius),
            Real::from(cy + tangent_length),
        );
        let line = sketch.add_line_segment2("line", start, line_end);
        let handle = sketch.add_arc_line_tangent2(
            "arc line tangent",
            arc,
            SketchArcEndpoint::Start,
            line,
            SketchLineEndpoint::Start,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 5);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ArcLineTangent)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(forms.forms[4].role, SketchResidualFormRole::ExactProof);
        let orientation = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(orientation.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_arc_line_tangent_rows_match_generated_integer_circle_tangents(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
        tangent_length in 1_i16..=12,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let tangent_length = i64::from(tangent_length);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let line_start = sketch.add_point3d(
            "line start",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + z0),
        );
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + tangent_length),
            Real::from(oz + z1),
        );
        let line = sketch.add_line_segment3("line", line_start, line_end);
        let handle = sketch.add_projected_arc_line_tangent3(
            "projected arc line tangent",
            workplane,
            arc,
            SketchArcEndpoint::Start,
            line,
            SketchLineEndpoint::Start,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 6);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_tangent_rows_generated = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedArcLineTangent)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_tangent_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in &forms.forms[..5] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(forms.forms[5].role, SketchResidualFormRole::ExactProof);
        let orientation = forms.forms[5].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(orientation.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_arc_cubic_tangent_rows_match_generated_integer_circle_tangents(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
        tangent_step in 1_i16..=6,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let tangent_step = i64::from(tangent_step);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let p1 = sketch.add_point2d(
            "p1",
            Real::from(cx + radius),
            Real::from(cy + tangent_step),
        );
        let p2 = sketch.add_point2d(
            "p2",
            Real::from(cx + radius),
            Real::from(cy + 2 * tangent_step),
        );
        let p3 = sketch.add_point2d(
            "p3",
            Real::from(cx + radius),
            Real::from(cy + 3 * tangent_step),
        );
        let cubic = sketch.add_cubic2("cubic", start, p1, p2, p3);
        let parameter = sketch.add_parameter("t", Real::from(0));
        let handle = sketch.add_arc_cubic_tangent2(
            "arc cubic tangent",
            arc,
            SketchArcEndpoint::Start,
            cubic,
            parameter,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 5);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ArcCubicTangent)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ArcCubicTangentOrientationPredicate
        );
        let orientation = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(orientation.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_arc_cubic_curve_tangent_rows_match_projected_circle_tangents(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
        tangent_step in 1_i16..=6,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let tangent_step = i64::from(tangent_step);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let p0 = sketch.add_point3d(
            "p0",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + z0),
        );
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + tangent_step),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + 2 * tangent_step),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + 3 * tangent_step),
            Real::from(oz + z3),
        );
        let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
        let parameter = sketch.add_parameter("t", Real::zero());
        let handle = sketch.add_projected_arc_cubic_curve_tangent3(
            "projected arc cubic tangent",
            workplane,
            arc,
            SketchArcEndpoint::Start,
            cubic,
            parameter,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 6);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_tangent_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedArcCubicCurveTangent)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_projected_tangent_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in &forms.forms[..5] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ProjectedArcCubicCurveTangentOrientationPredicate
        );
        let orientation = forms.forms[5].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(orientation.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_arc_cubic_curve_second_order_contact_rows_match_projected_circle_jets(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=8,
        z0 in -6_i16..=6,
        z1 in -6_i16..=6,
        z2 in -6_i16..=6,
        z3 in -6_i16..=6,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::one(), Real::zero(), Real::zero(), Real::zero());
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let tangent_step = 10 * radius;
        let p0 = sketch.add_point3d(
            "p0",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + z0),
        );
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + tangent_step),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + cx - 149 * radius),
            Real::from(oy + cy + 2 * tangent_step),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + cx - 299 * radius),
            Real::from(oy + cy + 3 * tangent_step),
            Real::from(oz + z3),
        );
        let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
        let parameter = sketch.add_parameter("t", Real::zero());
        let handle = sketch.add_projected_arc_cubic_curve_second_order_contact3(
            "projected arc cubic second order contact",
            workplane,
            arc,
            SketchArcEndpoint::Start,
            cubic,
            parameter,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 7);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_contact_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedArcCubicCurveSecondOrderContact)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_projected_contact_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 7);
        for form in &forms.forms[..5] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ProjectedArcCubicCurveSecondOrderOrientationPredicate
        );
        let orientation = forms.forms[5].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(orientation.structural_facts().sign, Some(RealSign::Positive));
        prop_assert_eq!(
            forms.forms[6].kind,
            SketchResidualFormKind::ProjectedArcCubicCurveSecondOrderContactPolynomial
        );
        prop_assert_eq!(
            forms.forms[6].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }

    #[test]
    fn sketch_arc_cubic_second_order_contact_rows_match_generated_circle_jets(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let p1 = sketch.add_point2d(
            "p1",
            Real::from(cx + radius),
            Real::from(cy + 2 * radius),
        );
        let p2 = sketch.add_point2d(
            "p2",
            Real::from(cx - 5 * radius),
            Real::from(cy + 4 * radius),
        );
        let p3 = sketch.add_point2d(
            "p3",
            Real::from(cx - 11 * radius),
            Real::from(cy + 6 * radius),
        );
        let cubic = sketch.add_cubic2("cubic", start, p1, p2, p3);
        let parameter = sketch.add_parameter("t", Real::from(0));
        let handle = sketch.add_arc_cubic_second_order_contact2(
            "arc cubic second order",
            arc,
            SketchArcEndpoint::Start,
            cubic,
            parameter,
            SketchTangentOrientation::CounterClockwise,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 6);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ArcCubicSecondOrderContact)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ArcCubicSecondOrderContactPolynomial
        );
        prop_assert_eq!(
            forms.forms[5].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }

    #[test]
    fn sketch_arc_arc_tangent_rows_match_generated_external_circle_tangents(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let mut sketch = SketchSolveProblem::new();
        let first_center = sketch.add_point2d("first center", Real::from(cx), Real::from(cy));
        let shared = sketch.add_point2d("shared", Real::from(cx + radius), Real::from(cy));
        let first_end = sketch.add_point2d("first end", Real::from(cx), Real::from(cy + radius));
        let first_radius = sketch.add_distance("first radius", Real::from(radius));
        let first = sketch.add_arc_of_circle2(
            "first arc",
            first_center,
            shared,
            first_end,
            first_radius,
        );
        let second_center =
            sketch.add_point2d("second center", Real::from(cx + 2 * radius), Real::from(cy));
        let second_end = sketch.add_point2d(
            "second end",
            Real::from(cx + 2 * radius),
            Real::from(cy + radius),
        );
        let second_radius = sketch.add_distance("second radius", Real::from(radius));
        let second = sketch.add_arc_of_circle2(
            "second arc",
            second_center,
            shared,
            second_end,
            second_radius,
        );
        let handle = sketch.add_arc_arc_tangent2(
            "arc arc tangent",
            first,
            SketchArcEndpoint::Start,
            second,
            SketchArcEndpoint::Start,
            SketchArcTangencyBranch::OppositeRadiusDirection,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 6);
        let all_rows_generated = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ArcArcTangent)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_rows_generated);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in &forms.forms[..5] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ArcArcTangentRadiusBranchPredicate
        );
        let branch = forms.forms[5].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(branch.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_point_line_residual_forms_replay_generated_signed_branches(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        px in -8_i16..=8,
        offset in -12_i16..=12,
    ) {
        prop_assume!(offset != 0);
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let px = i64::from(px);
        let offset = i64::from(offset);
        let distance_value = offset.abs();
        let mut sketch = SketchSolveProblem::new();
        let start = sketch.add_point2d("start", Real::from(x0), Real::from(y0));
        let end = sketch.add_point2d("end", Real::from(x0 + 5), Real::from(y0));
        let point = sketch.add_point2d("point", Real::from(px), Real::from(y0 + offset));
        let line = sketch.add_line_segment2("line", start, end);
        let distance = sketch.add_distance("distance", Real::from(distance_value));
        let handle = sketch.add_point_line_distance2("point line distance", point, line, distance);

        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&sketch.lower_to_problem().problem);

        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::SquaredPointLineDistancePolynomial
        );
        prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(
            forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
        let positive_branch = forms.forms[1].residual.eval_real(context.bindings()).unwrap();
        let negative_branch = forms.forms[2].residual.eval_real(context.bindings()).unwrap();
        if offset < 0 {
            prop_assert_eq!(positive_branch, Real::zero());
            prop_assert_ne!(negative_branch, Real::zero());
        } else {
            prop_assert_ne!(positive_branch, Real::zero());
            prop_assert_eq!(negative_branch, Real::zero());
        }
    }

    #[test]
    fn sketch_point_on_circle_residual_forms_replay_generated_integer_triangles(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        sx in prop_oneof![Just(-1_i16), Just(1_i16)],
        sy in prop_oneof![Just(-1_i16), Just(1_i16)],
        scale in 1_i16..=4,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let scale = i64::from(scale);
        let dx = 3 * scale * sx;
        let dy = 4 * scale * sy;
        let radius = 5 * scale;
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let point = sketch.add_point2d("point", Real::from(cx + dx), Real::from(cy + dy));
        let radius = sketch.add_distance("radius", Real::from(radius));
        let circle = sketch.add_circle2("circle", center, radius);
        let handle = sketch.add_point_on_circle("point on circle", point, circle);

        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&sketch.lower_to_problem().problem);

        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::SquaredCircleIncidencePolynomial
        );
        prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(
            forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::CircleRadialDistanceProposal
        );
        prop_assert_eq!(forms.forms[1].role, SketchResidualFormRole::ProposalOnly);
        prop_assert_eq!(
            forms.forms[1].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }

    #[test]
    fn sketch_point_on_line_rows_replay_generated_collinear_points(
        sx in -8_i16..=8,
        sy in -8_i16..=8,
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        t in -8_i16..=8,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let t = i64::from(t);
        let mut sketch = SketchSolveProblem::new();
        let start = sketch.add_point2d("start", Real::from(sx), Real::from(sy));
        let end = sketch.add_point2d("end", Real::from(sx + ux), Real::from(sy + uy));
        let line = sketch.add_line_segment2("line", start, end);
        let point =
            sketch.add_point2d("point", Real::from(sx + t * ux), Real::from(sy + t * uy));
        let handle = sketch.add_point_on_line2("point on line", point, line);

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::PointLineIncidence));
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 1);
        prop_assert_eq!(forms.forms[0].kind, SketchResidualFormKind::PointLineIncidencePolynomial);
        prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(
            forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }

    #[test]
    fn sketch_point_on_cubic_rows_replay_generated_affine_cubics(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let mut sketch = SketchSolveProblem::new();
        let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
        let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
        let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
        let point = sketch.add_point2d(
            "point",
            Real::from(ax + 3 * dx * t),
            Real::from(ay + 3 * dy * t),
        );
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle = sketch.add_point_on_cubic2("point on cubic", point, cubic, parameter);

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        for form in &forms.forms {
            prop_assert_eq!(form.kind, SketchResidualFormKind::CubicBezierIncidencePolynomial);
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
    }

    #[test]
    fn sketch_projected_point_on_cubic_rows_replay_identity_workplane_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z = i64::from(z);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
        let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
        let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + ax + 3 * dx * t),
            Real::from(oy + ay + 3 * dy * t),
            Real::from(oz + z),
        );
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle =
            sketch.add_projected_point_on_cubic3("projected point on cubic", workplane, point, cubic, parameter);

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_projected_cubic_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicBezierIncidence)
        });
        prop_assert!(all_projected_cubic_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
        );
        for form in &forms.forms[1..] {
            prop_assert_eq!(form.kind, SketchResidualFormKind::ProjectedCubicBezierIncidencePolynomial);
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
    }

    #[test]
    fn sketch_projected_point_on_cubic_curve3_rows_replay_projected_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        point_z in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let point_z = i64::from(point_z);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point3d("p0", Real::from(ox + ax), Real::from(oy + ay), Real::from(oz + z0));
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + ax + dx),
            Real::from(oy + ay + dy),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + ax + 2 * dx),
            Real::from(oy + ay + 2 * dy),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + ax + 3 * dx),
            Real::from(oy + ay + 3 * dy),
            Real::from(oz + z3),
        );
        let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + ax + 3 * dx * t),
            Real::from(oy + ay + 3 * dy * t),
            Real::from(oz + point_z),
        );
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle = sketch.add_projected_point_on_cubic_curve3(
            "projected point on cubic curve",
            workplane,
            point,
            cubic,
            parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_cubic_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveIncidence)
        });
        prop_assert!(all_projected_cubic_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
        );
        for form in &forms.forms[1..] {
            prop_assert_eq!(form.kind, SketchResidualFormKind::ProjectedCubicCurveIncidencePolynomial);
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
    }

    #[test]
    fn sketch_cubic_line_tangent_rows_match_generated_affine_cubics(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        tangent_scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let tangent_scale = i64::from(tangent_scale);
        let mut sketch = SketchSolveProblem::new();
        let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
        let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
        let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
        let point = sketch.add_point2d(
            "point",
            Real::from(ax + 3 * dx * t),
            Real::from(ay + 3 * dy * t),
        );
        let tangent_end = sketch.add_point2d(
            "tangent end",
            Real::from(ax + 3 * dx * t + dx * tangent_scale),
            Real::from(ay + 3 * dy * t + dy * tangent_scale),
        );
        let line = sketch.add_line_segment2("line", point, tangent_end);
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle = sketch.add_cubic_line_tangent2("cubic tangent", cubic, parameter, line, SketchLineEndpoint::Start);

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 4);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 4);
        for form in &forms.forms[..3] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[3].kind,
            SketchResidualFormKind::CubicLineTangentDotProductPredicate
        );
        let dot = forms.forms[3].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_cubic_line_tangent_rows_match_generated_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        tangent_scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let tangent_scale = i64::from(tangent_scale);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
        let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
        let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + ax + 3 * dx * t),
            Real::from(oy + ay + 3 * dy * t),
            Real::from(oz + z0),
        );
        let tangent_end = sketch.add_point3d(
            "tangent end",
            Real::from(ox + ax + 3 * dx * t + dx * tangent_scale),
            Real::from(oy + ay + 3 * dy * t + dy * tangent_scale),
            Real::from(oz + z0 + z1),
        );
        let line = sketch.add_line_segment3("line", point, tangent_end);
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle = sketch.add_projected_cubic_line_tangent3(
            "projected cubic tangent",
            workplane,
            cubic,
            parameter,
            line,
            SketchLineEndpoint::Start,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 5);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_tangent_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicLineTangent)
        });
        prop_assert!(all_projected_tangent_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
        );
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ProjectedCubicLineTangentDotProductPredicate
        );
        let dot = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_cubic_curve_line_tangent_rows_match_projected_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        line_z in -8_i16..=8,
        tangent_scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let line_z = i64::from(line_z);
        let tangent_scale = i64::from(tangent_scale);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point3d("p0", Real::from(ox + ax), Real::from(oy + ay), Real::from(oz + z0));
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + ax + dx),
            Real::from(oy + ay + dy),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + ax + 2 * dx),
            Real::from(oy + ay + 2 * dy),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + ax + 3 * dx),
            Real::from(oy + ay + 3 * dy),
            Real::from(oz + z3),
        );
        let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
        let endpoint = sketch.add_point3d(
            "endpoint",
            Real::from(ox + ax + 3 * dx * t),
            Real::from(oy + ay + 3 * dy * t),
            Real::from(oz + line_z),
        );
        let tangent_end = sketch.add_point3d(
            "tangent end",
            Real::from(ox + ax + 3 * dx * t + dx * tangent_scale),
            Real::from(oy + ay + 3 * dy * t + dy * tangent_scale),
            Real::from(oz + line_z + z0 - z3),
        );
        let line = sketch.add_line_segment3("line", endpoint, tangent_end);
        let parameter = sketch.add_parameter("t", Real::from(t));
        let handle = sketch.add_projected_cubic_curve_line_tangent3(
            "projected 3D cubic tangent",
            workplane,
            cubic,
            parameter,
            line,
            SketchLineEndpoint::Start,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 5);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_tangent_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveLineTangent)
        });
        prop_assert!(all_projected_tangent_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ProjectedCubicCurveLineTangentDotProductPredicate
        );
        let dot = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_cubic_curve_cubic_curve_tangent_rows_match_projected_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        qz0 in -8_i16..=8,
        qz1 in -8_i16..=8,
        qz2 in -8_i16..=8,
        qz3 in -8_i16..=8,
        tangent_scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let qz0 = i64::from(qz0);
        let qz1 = i64::from(qz1);
        let qz2 = i64::from(qz2);
        let qz3 = i64::from(qz3);
        let tangent_scale = i64::from(tangent_scale);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point3d("p0", Real::from(ox + ax), Real::from(oy + ay), Real::from(oz + z0));
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + ax + dx),
            Real::from(oy + ay + dy),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + ax + 2 * dx),
            Real::from(oy + ay + 2 * dy),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + ax + 3 * dx),
            Real::from(oy + ay + 3 * dy),
            Real::from(oz + z3),
        );
        let first = sketch.add_cubic3("first", p0, p1, p2, p3);
        let junction_x = ox + ax + 3 * dx * t;
        let junction_y = oy + ay + 3 * dy * t;
        let q0 = sketch.add_point3d(
            "q0",
            Real::from(junction_x),
            Real::from(junction_y),
            Real::from(oz + qz0),
        );
        let q1 = sketch.add_point3d(
            "q1",
            Real::from(junction_x + dx * tangent_scale),
            Real::from(junction_y + dy * tangent_scale),
            Real::from(oz + qz1),
        );
        let q2 = sketch.add_point3d(
            "q2",
            Real::from(junction_x + 2 * dx * tangent_scale),
            Real::from(junction_y + 2 * dy * tangent_scale),
            Real::from(oz + qz2),
        );
        let q3 = sketch.add_point3d(
            "q3",
            Real::from(junction_x + 3 * dx * tangent_scale),
            Real::from(junction_y + 3 * dy * tangent_scale),
            Real::from(oz + qz3),
        );
        let second = sketch.add_cubic3("second", q0, q1, q2, q3);
        let first_parameter = sketch.add_parameter("first t", Real::from(t));
        let second_parameter = sketch.add_parameter("second t", Real::zero());
        let handle = sketch.add_projected_cubic_curve_cubic_curve_tangent3(
            "projected 3D cubic/cubic tangent",
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 5);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_tangent_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveCubicCurveTangent)
        });
        prop_assert!(all_projected_tangent_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        for form in &forms.forms[..4] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ProjectedCubicCurveCubicCurveTangentDotProductPredicate
        );
        let dot = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_projected_cubic_curve_cubic_curve_c2_rows_match_projected_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        qz0 in -8_i16..=8,
        qz1 in -8_i16..=8,
        qz2 in -8_i16..=8,
        qz3 in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let qz0 = i64::from(qz0);
        let qz1 = i64::from(qz1);
        let qz2 = i64::from(qz2);
        let qz3 = i64::from(qz3);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point3d("p0", Real::from(ox + ax), Real::from(oy + ay), Real::from(oz + z0));
        let p1 = sketch.add_point3d(
            "p1",
            Real::from(ox + ax + dx),
            Real::from(oy + ay + dy),
            Real::from(oz + z1),
        );
        let p2 = sketch.add_point3d(
            "p2",
            Real::from(ox + ax + 2 * dx),
            Real::from(oy + ay + 2 * dy),
            Real::from(oz + z2),
        );
        let p3 = sketch.add_point3d(
            "p3",
            Real::from(ox + ax + 3 * dx),
            Real::from(oy + ay + 3 * dy),
            Real::from(oz + z3),
        );
        let first = sketch.add_cubic3("first", p0, p1, p2, p3);
        let junction_x = ox + ax + 3 * dx * t;
        let junction_y = oy + ay + 3 * dy * t;
        let q0 = sketch.add_point3d(
            "q0",
            Real::from(junction_x),
            Real::from(junction_y),
            Real::from(oz + qz0),
        );
        let q1 = sketch.add_point3d(
            "q1",
            Real::from(junction_x + dx),
            Real::from(junction_y + dy),
            Real::from(oz + qz1),
        );
        let q2 = sketch.add_point3d(
            "q2",
            Real::from(junction_x + 2 * dx),
            Real::from(junction_y + 2 * dy),
            Real::from(oz + qz2),
        );
        let q3 = sketch.add_point3d(
            "q3",
            Real::from(junction_x + 3 * dx),
            Real::from(junction_y + 3 * dy),
            Real::from(oz + qz3),
        );
        let second = sketch.add_cubic3("second", q0, q1, q2, q3);
        let first_parameter = sketch.add_parameter("first t", Real::from(t));
        let second_parameter = sketch.add_parameter("second t", Real::zero());
        let handle = sketch.add_projected_cubic_curve_cubic_curve_c2_continuity3(
            "projected 3D cubic/cubic c2",
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 7);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        let all_projected_c2_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveCubicCurveC2Continuity)
        });
        prop_assert!(all_projected_c2_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 7);
        for form in &forms.forms {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                form.residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ProjectedCubicCurveCubicCurveC2SecondDerivativePolynomial
        );
    }

    #[test]
    fn sketch_projected_cubic_curve_cubic_curve_g2_rows_match_projected_affine_cubics(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        qz1 in -8_i16..=8,
        qz2 in -8_i16..=8,
        qz3 in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let qz1 = i64::from(qz1);
        let qz2 = i64::from(qz2);
        let qz3 = i64::from(qz3);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let p0 = sketch.add_point3d("p0", Real::from(ox + ax), Real::from(oy + ay), Real::from(oz + z0));
        let p1 = sketch.add_point3d("p1", Real::from(ox + ax + dx), Real::from(oy + ay + dy), Real::from(oz + z1));
        let p2 = sketch.add_point3d("p2", Real::from(ox + ax + 2 * dx), Real::from(oy + ay + 2 * dy), Real::from(oz + z2));
        let p3 = sketch.add_point3d("p3", Real::from(ox + ax + 3 * dx), Real::from(oy + ay + 3 * dy), Real::from(oz + z3));
        let q1 = sketch.add_point3d("q1", Real::from(ox + ax + 4 * dx), Real::from(oy + ay + 4 * dy), Real::from(oz + qz1));
        let q2 = sketch.add_point3d("q2", Real::from(ox + ax + 5 * dx), Real::from(oy + ay + 5 * dy), Real::from(oz + qz2));
        let q3 = sketch.add_point3d("q3", Real::from(ox + ax + 6 * dx), Real::from(oy + ay + 6 * dy), Real::from(oz + qz3));
        let first = sketch.add_cubic3("first", p0, p1, p2, p3);
        let second = sketch.add_cubic3("second", p3, q1, q2, q3);
        let first_parameter = sketch.add_parameter("first t", Real::from(1));
        let second_parameter = sketch.add_parameter("second t", Real::zero());
        let handle = sketch.add_projected_cubic_curve_cubic_curve_g2_continuity3(
            "projected 3D cubic/cubic g2",
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 7);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_projected_g2_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveCubicCurveG2Continuity)
        });
        prop_assert!(all_projected_g2_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 7);
        for form in [&forms.forms[0], &forms.forms[1], &forms.forms[2], &forms.forms[3], &forms.forms[5], &forms.forms[6]] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        let dot = forms.forms[4].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ProjectedCubicCurveCubicCurveG2CurvatureMagnitudePolynomial
        );
    }

    #[test]
    fn sketch_cubic_cubic_tangent_rows_match_generated_affine_cubics(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
        t in -2_i16..=2,
        second_scale in 1_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let t = i64::from(t);
        let second_scale = i64::from(second_scale);
        let join_x = ax + 3 * dx * t;
        let join_y = ay + 3 * dy * t;
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let a2 = sketch.add_point2d("a2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let a3 = sketch.add_point2d("a3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let b0 = sketch.add_point2d("b0", Real::from(join_x), Real::from(join_y));
        let b1 = sketch.add_point2d(
            "b1",
            Real::from(join_x + dx * second_scale),
            Real::from(join_y + dy * second_scale),
        );
        let b2 = sketch.add_point2d(
            "b2",
            Real::from(join_x + 2 * dx * second_scale),
            Real::from(join_y + 2 * dy * second_scale),
        );
        let b3 = sketch.add_point2d(
            "b3",
            Real::from(join_x + 3 * dx * second_scale),
            Real::from(join_y + 3 * dy * second_scale),
        );
        let first = sketch.add_cubic2("first", a0, a1, a2, a3);
        let second = sketch.add_cubic2("second", b0, b1, b2, b3);
        let first_parameter = sketch.add_parameter("first t", Real::from(t));
        let second_parameter = sketch.add_parameter("second t", Real::from(0));
        let handle = sketch.add_cubic_cubic_tangent2(
            "cubic cubic tangent",
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 4);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 4);
        for form in &forms.forms[..3] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[3].kind,
            SketchResidualFormKind::CubicCubicTangentDotProductPredicate
        );
        let dot = forms.forms[3].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_cubic_cubic_g2_rows_match_generated_affine_cubics(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let a2 = sketch.add_point2d("a2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let join = sketch.add_point2d("join", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let b1 = sketch.add_point2d("b1", Real::from(ax + 4 * dx), Real::from(ay + 4 * dy));
        let b2 = sketch.add_point2d("b2", Real::from(ax + 5 * dx), Real::from(ay + 5 * dy));
        let b3 = sketch.add_point2d("b3", Real::from(ax + 6 * dx), Real::from(ay + 6 * dy));
        let first = sketch.add_cubic2("first", a0, a1, a2, join);
        let second = sketch.add_cubic2("second", join, b1, b2, b3);
        let first_parameter = sketch.add_parameter("first t", Real::from(1));
        let second_parameter = sketch.add_parameter("second t", Real::from(0));
        let handle = sketch.add_cubic_cubic_g2_continuity2(
            "cubic g2",
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 6);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in [&forms.forms[0], &forms.forms[1], &forms.forms[2], &forms.forms[4], &forms.forms[5]] {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        let dot = forms.forms[3].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(dot.structural_facts().sign, Some(RealSign::Positive));
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::CubicCubicG2CurvatureMagnitudePolynomial
        );
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::CubicCubicG2CurvatureSignPredicate
        );
    }

    #[test]
    fn sketch_cubic_cubic_c2_rows_match_generated_affine_cubics(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        dx in -4_i16..=4,
        dy in -4_i16..=4,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
        let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
        let a2 = sketch.add_point2d("a2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
        let join = sketch.add_point2d("join", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
        let b1 = sketch.add_point2d("b1", Real::from(ax + 4 * dx), Real::from(ay + 4 * dy));
        let b2 = sketch.add_point2d("b2", Real::from(ax + 5 * dx), Real::from(ay + 5 * dy));
        let b3 = sketch.add_point2d("b3", Real::from(ax + 6 * dx), Real::from(ay + 6 * dy));
        let first = sketch.add_cubic2("first", a0, a1, a2, join);
        let second = sketch.add_cubic2("second", join, b1, b2, b3);
        let first_parameter = sketch.add_parameter("first t", Real::from(1));
        let second_parameter = sketch.add_parameter("second t", Real::from(0));
        let handle = sketch.add_cubic_cubic_c2_continuity2(
            "cubic c2",
            first,
            first_parameter,
            second,
            second_parameter,
        );

        let lowered = sketch.lower_to_problem();
        let forms = sketch.residual_forms_for_constraint(handle);
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);

        prop_assert!(lowered.all_generated());
        prop_assert_eq!(lowered.problem.constraints.len(), 6);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        for form in &forms.forms {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::CubicCubicC2SecondDerivativePolynomial
        );
    }

    #[test]
    fn sketch_equal_angle_rows_match_generated_scaled_line_pairs(
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        vx in -6_i16..=6,
        vy in -6_i16..=6,
        u_scale in 1_i16..=4,
        v_scale in 1_i16..=4,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        prop_assume!(vx != 0 || vy != 0);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let vx = i64::from(vx);
        let vy = i64::from(vy);
        let u_scale = i64::from(u_scale);
        let v_scale = i64::from(v_scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(0), Real::from(0));
        let a1 = sketch.add_point2d("a1", Real::from(ux), Real::from(uy));
        let b0 = sketch.add_point2d("b0", Real::from(0), Real::from(0));
        let b1 = sketch.add_point2d("b1", Real::from(vx), Real::from(vy));
        let c0 = sketch.add_point2d("c0", Real::from(3), Real::from(-2));
        let c1 = sketch.add_point2d(
            "c1",
            Real::from(3 + u_scale * ux),
            Real::from(-2 + u_scale * uy),
        );
        let d0 = sketch.add_point2d("d0", Real::from(-5), Real::from(4));
        let d1 = sketch.add_point2d(
            "d1",
            Real::from(-5 + v_scale * vx),
            Real::from(4 + v_scale * vy),
        );
        let a = sketch.add_line_segment2("a", a0, a1);
        let b = sketch.add_line_segment2("b", b0, b1);
        let c = sketch.add_line_segment2("c", c0, c1);
        let d = sketch.add_line_segment2("d", d0, d1);
        sketch.add_equal_angle_lines2("equal angle", a, b, c, d);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredCosineAngleEquality)
        );
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_oriented_angle_rows_match_generated_positive_scaled_line_pairs(
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        vx in -6_i16..=6,
        vy in -6_i16..=6,
        u_scale in 1_i16..=4,
        v_scale in 1_i16..=4,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        prop_assume!(vx != 0 || vy != 0);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let vx = i64::from(vx);
        let vy = i64::from(vy);
        let u_scale = i64::from(u_scale);
        let v_scale = i64::from(v_scale);
        let mut sketch = SketchSolveProblem::new();
        let a0 = sketch.add_point2d("a0", Real::from(0), Real::from(0));
        let a1 = sketch.add_point2d("a1", Real::from(ux), Real::from(uy));
        let b0 = sketch.add_point2d("b0", Real::from(0), Real::from(0));
        let b1 = sketch.add_point2d("b1", Real::from(vx), Real::from(vy));
        let c0 = sketch.add_point2d("c0", Real::from(3), Real::from(-2));
        let c1 = sketch.add_point2d(
            "c1",
            Real::from(3 + u_scale * ux),
            Real::from(-2 + u_scale * uy),
        );
        let d0 = sketch.add_point2d("d0", Real::from(-5), Real::from(4));
        let d1 = sketch.add_point2d(
            "d1",
            Real::from(-5 + v_scale * vx),
            Real::from(4 + v_scale * vy),
        );
        let a = sketch.add_line_segment2("a", a0, a1);
        let b = sketch.add_line_segment2("b", b0, b1);
        let c = sketch.add_line_segment2("c", c0, c1);
        let d = sketch.add_line_segment2("d", d0, d1);
        let handle = sketch.add_equal_oriented_angle_lines2("oriented angle", a, b, c, d);

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_oriented_angle_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::OrientedAngleEquality)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_oriented_angle_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[0].kind,
            SketchResidualFormKind::OrientedAngleVectorCollinearityPolynomial
        );
        prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
        prop_assert_eq!(
            forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::OrientedAngleSameBranchPredicate
        );
        prop_assert_eq!(forms.forms[1].role, SketchResidualFormRole::ExactProof);
        let branch = forms.forms[1].residual.eval_real(context.bindings()).unwrap();
        prop_assert_eq!(branch.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn sketch_midpoint_rows_match_generated_integer_points(
        mx in -12_i16..=12,
        my in -12_i16..=12,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
    ) {
        let mx = i64::from(mx);
        let my = i64::from(my);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let midpoint = sketch.add_point2d("midpoint", Real::from(mx), Real::from(my));
        let a = sketch.add_point2d("a", Real::from(mx - dx), Real::from(my - dy));
        let b = sketch.add_point2d("b", Real::from(mx + dx), Real::from(my + dy));
        sketch.add_at_midpoint2("midpoint relation", midpoint, a, b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_midpoint_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::MidpointCoordinateEquality)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_midpoint_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_axis_symmetry_rows_match_generated_integer_reflections(
        axis_x in -8_i16..=8,
        axis_y in -8_i16..=8,
        px in -8_i16..=8,
        py in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
    ) {
        let axis_x = i64::from(axis_x);
        let axis_y = i64::from(axis_y);
        let px = i64::from(px);
        let py = i64::from(py);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let horizontal_a = sketch.add_point2d("ha", Real::from(px), Real::from(axis_y + dy));
        let horizontal_b = sketch.add_point2d("hb", Real::from(px), Real::from(axis_y - dy));
        let vertical_a = sketch.add_point2d("va", Real::from(axis_x + dx), Real::from(py));
        let vertical_b = sketch.add_point2d("vb", Real::from(axis_x - dx), Real::from(py));
        sketch.add_symmetric_horizontal2(
            "horizontal symmetry",
            horizontal_a,
            horizontal_b,
            Real::from(axis_y),
        );
        sketch.add_symmetric_vertical2(
            "vertical symmetry",
            vertical_a,
            vertical_b,
            Real::from(axis_x),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 4);
        let all_symmetry_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::AxisSymmetryCoordinateEquality)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_symmetry_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_line_symmetry_rows_match_generated_integer_reflections(
        sx in -8_i16..=8,
        sy in -8_i16..=8,
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        t in -5_i16..=5,
        offset in -5_i16..=5,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let t = i64::from(t);
        let offset = i64::from(offset);
        let midpoint_x = sx + t * ux;
        let midpoint_y = sy + t * uy;
        let normal_x = -uy;
        let normal_y = ux;
        let mut sketch = SketchSolveProblem::new();
        let axis_start = sketch.add_point2d("axis start", Real::from(sx), Real::from(sy));
        let axis_end =
            sketch.add_point2d("axis end", Real::from(sx + ux), Real::from(sy + uy));
        let axis = sketch.add_line_segment2("axis", axis_start, axis_end);
        let a = sketch.add_point2d(
            "a",
            Real::from(midpoint_x + offset * normal_x),
            Real::from(midpoint_y + offset * normal_y),
        );
        let b = sketch.add_point2d(
            "b",
            Real::from(midpoint_x - offset * normal_x),
            Real::from(midpoint_y - offset * normal_y),
        );
        sketch.add_symmetric_line2("line symmetry", a, b, axis);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_line_symmetry_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::LineSymmetryPolynomial)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_line_symmetry_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_projected_line_symmetry_rows_match_generated_integer_reflections(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        sx in -8_i16..=8,
        sy in -8_i16..=8,
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        t in -5_i16..=5,
        offset in -5_i16..=5,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let t = i64::from(t);
        let offset = i64::from(offset);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let midpoint_x = sx + t * ux;
        let midpoint_y = sy + t * uy;
        let normal_x = -uy;
        let normal_y = ux;
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let axis_start =
            sketch.add_point3d("axis start", Real::from(ox + sx), Real::from(oy + sy), Real::from(oz + z0));
        let axis_end = sketch.add_point3d(
            "axis end",
            Real::from(ox + sx + ux),
            Real::from(oy + sy + uy),
            Real::from(oz + z1),
        );
        let axis = sketch.add_line_segment3("axis", axis_start, axis_end);
        let a = sketch.add_point3d(
            "a",
            Real::from(ox + midpoint_x + offset * normal_x),
            Real::from(oy + midpoint_y + offset * normal_y),
            Real::from(oz + z0 + z1),
        );
        let b = sketch.add_point3d(
            "b",
            Real::from(ox + midpoint_x - offset * normal_x),
            Real::from(oy + midpoint_y - offset * normal_y),
            Real::from(oz - z0 - z1),
        );
        let handle = sketch.add_projected_symmetric_line3(
            "projected line symmetry",
            workplane,
            a,
            b,
            axis,
        );

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 3);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_projected_symmetry_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ProjectedLineSymmetryPolynomial)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_projected_symmetry_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        for form in &forms.forms {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
    }

    #[test]
    fn sketch_workplane_symmetry_rows_match_generated_integer_reflections(
        x in -12_i16..=12,
        y in -12_i16..=12,
        z0 in -12_i16..=12,
        offset in -12_i16..=12,
    ) {
        let x = i64::from(x);
        let y = i64::from(y);
        let z0 = i64::from(z0);
        let offset = i64::from(offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(z0));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d(
            "a",
            Real::from(x),
            Real::from(y),
            Real::from(z0 + offset),
        );
        let b = sketch.add_point3d(
            "b",
            Real::from(x),
            Real::from(y),
            Real::from(z0 - offset),
        );
        let handle = sketch.add_symmetric_workplane3("workplane symmetry", a, b, workplane);

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 5);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let all_workplane_symmetry_rows = lowered.rows[1..].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_workplane_symmetry_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 5);
        for form in &forms.forms {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::WorkplaneSymmetryMidpointPlanePolynomial
        );
        let all_normal_offset_forms = forms.forms[2..].iter().all(|form| {
            form.kind == SketchResidualFormKind::WorkplaneSymmetryNormalOffsetPolynomial
        });
        prop_assert!(all_normal_offset_forms);
    }

    #[test]
    fn sketch_construction_certificate_generated_integer_segments_match_distance(
        ax in -12_i16..=12,
        ay in -12_i16..=12,
        dx in -8_i16..=8,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
        let b = sketch.add_point2d("b", Real::from(ax + dx), Real::from(ay));
        let d = sketch.add_distance("d", Real::from(dx.abs()));
        sketch.add_point_point_distance("distance", a, b, d);

        let certificate = certify_sketch_construction(&sketch);

        prop_assert_eq!(
            certificate.status,
            SketchConstructionCertificateStatus::Certified
        );
        prop_assert!(certificate.is_certified());
        prop_assert!(certificate.residual_replay.all_satisfied());
    }

    #[test]
    fn sketch_generated_parameter_ranges_match_integer_bounds(
        value in -32_i16..=32,
        lower_offset in 0_i16..=16,
        upper_offset in 0_i16..=16,
    ) {
        let value = i64::from(value);
        let lower = value - i64::from(lower_offset);
        let upper = value + i64::from(upper_offset);
        let mut sketch = SketchSolveProblem::new();
        let parameter = sketch.add_parameter("t", hyperreal::Real::from(value));
        sketch_range_builders::parameter_range(
            &mut sketch,
            "range",
            parameter,
            Some(hyperreal::Real::from(lower)),
            Some(hyperreal::Real::from(upper)),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 2);
        let all_range_rows = lowered.rows.iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::ParameterRange)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_range_rows);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn sketch_generated_parameter_ordering_matches_integer_order(
        lower in -32_i16..=32,
        upper in -32_i16..=32,
    ) {
        let lower = i64::from(lower);
        let upper = i64::from(upper);
        let mut sketch = SketchSolveProblem::new();
        let lower_parameter = sketch.add_parameter("lower", Real::from(lower));
        let upper_parameter = sketch.add_parameter("upper", Real::from(upper));
        sketch.add_parameter_ordering("nondecreasing", lower_parameter, upper_parameter);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::ParameterOrdering)
        );
        prop_assert_eq!(
            certification.all_satisfied(),
            upper >= lower
        );
    }

    #[test]
    fn sketch_generated_parameter_margins_match_integer_separation(
        lower in -32_i16..=32,
        gap in 0_i16..=32,
        margin in 0_i16..=32,
    ) {
        let lower = i64::from(lower);
        let upper = lower + i64::from(gap);
        let margin = i64::from(margin);
        let mut sketch = SketchSolveProblem::new();
        let lower_parameter = sketch.add_parameter("lower", Real::from(lower));
        let upper_parameter = sketch.add_parameter("upper", Real::from(upper));
        sketch.add_parameter_margin(
            "margin",
            lower_parameter,
            upper_parameter,
            Real::from(margin),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.rows.len(), 1);
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::ParameterMargin)
        );
        prop_assert_eq!(certification.all_satisfied(), i64::from(gap) >= margin);
    }

    #[test]
    fn sketch_parameter_domain_preflight_generated_closed_bounds_match_integer_order(
        value in -32_i16..=32,
        lower in -32_i16..=32,
        upper in -32_i16..=32,
    ) {
        let value = i64::from(value);
        let lower = i64::from(lower);
        let upper = i64::from(upper);
        let mut sketch = SketchSolveProblem::new();
        let parameter = sketch.add_parameter("bounded", Real::from(value));
        let domain_added = sketch.add_parameter_domain(
            parameter,
            SketchParameterDomain::Bounded {
                lower: Some(Real::from(lower)),
                upper: Some(Real::from(upper)),
            },
        );
        prop_assert!(domain_added);

        let report = preflight_sketch_parameter_domains(&sketch);
        let expected_valid = lower <= upper && lower <= value && value <= upper;

        prop_assert_eq!(report.checks.len(), 1);
        prop_assert_eq!(
            report.checks[0].status == SketchParameterDomainStatus::CertifiedValid,
            expected_valid
        );
        prop_assert_eq!(
            report.checks[0].status == SketchParameterDomainStatus::CertifiedInvalid,
            !expected_valid
        );
    }

    #[test]
    fn sketch_unit_tolerance_audit_generated_tolerances_follow_exact_sign(
        tolerance in -32_i16..=32,
        has_unit in any::<bool>(),
    ) {
        let tolerance = i64::from(tolerance);
        let mut sketch = SketchSolveProblem::new();
        let parameter = sketch.add_parameter("toleranced", Real::from(0));
        let metadata = SketchRoundTripMetadata {
            source_unit: has_unit.then(|| "mm".to_owned()),
            declared_tolerance: Some(Real::from(tolerance)),
            ..SketchRoundTripMetadata::default()
        };
        let metadata_set = sketch.set_parameter_metadata(parameter, metadata);
        prop_assert!(metadata_set);

        let report = audit_sketch_unit_tolerances(&sketch);
        let row = report
            .rows
            .iter()
            .find(|row| row.parameter == Some(parameter))
            .expect("parameter unit/tolerance row should exist");
        let expected = if tolerance < 0 {
            SketchUnitToleranceStatus::CertifiedInvalidNegativeTolerance
        } else if has_unit {
            SketchUnitToleranceStatus::CertifiedDeclaredTolerance
        } else {
            SketchUnitToleranceStatus::ToleranceWithoutUnit
        };

        prop_assert_eq!(&row.status, &expected);
    }

    #[test]
    fn sketch_degeneracy_preflight_generated_line_lengths_match_integer_points(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        bx in -16_i16..=16,
        by in -16_i16..=16,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
        let b = sketch.add_point2d("b", Real::from(bx), Real::from(by));
        let line = sketch.add_line_segment2("line", a, b);

        let report = preflight_sketch_degeneracies(&sketch);
        let row = report
            .checks
            .iter()
            .find(|check| check.entity == line && check.kind == SketchDegeneracyKind::ZeroLengthLineSegment2)
            .expect("line degeneracy row should be emitted");
        let expected_degenerate = ax == bx && ay == by;

        prop_assert_eq!(
            row.status == SketchDegeneracyStatus::CertifiedDegenerate,
            expected_degenerate
        );
        prop_assert_eq!(
            row.status == SketchDegeneracyStatus::CertifiedNondegenerate,
            !expected_degenerate
        );
    }

    #[test]
    fn sketch_workplane_lift_generated_identity_quaternion_matches_integer_uv(
        ox in -16_i16..=16,
        oy in -16_i16..=16,
        oz in -16_i16..=16,
        u in -16_i16..=16,
        v in -16_i16..=16,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let u = i64::from(u);
        let v = i64::from(v);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let point = sketch.add_point2d("uv", Real::from(u), Real::from(v));

        let report = lift_sketch_point2_to_workplane3(&sketch, workplane, point);

        prop_assert_eq!(report.status, SketchWorkplaneFrameStatus::Certified);
        prop_assert_eq!(
            report.lifted_coordinates,
            Some([Real::from(ox + u), Real::from(oy + v), Real::from(oz)])
        );
    }

    #[test]
    fn sketch_projected_distance_identity_workplane_ignores_normal_displacement(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        az in -16_i16..=16,
        u in -16_i16..=16,
        normal_offset in -16_i16..=16,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let az = i64::from(az);
        let u = i64::from(u);
        let normal_offset = i64::from(normal_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", Real::from(ax), Real::from(ay), Real::from(az));
        let b = sketch.add_point3d(
            "b",
            Real::from(ax + u),
            Real::from(ay),
            Real::from(az + normal_offset),
        );
        let distance = sketch.add_distance("projected distance", Real::from(u.abs()));
        sketch.add_projected_point_point_distance("projected", workplane, a, b, distance);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedDistance)
        );
    }

    #[test]
    fn sketch_projected_equal_point_point_distances3_identity_workplane_ignores_normal_offsets(
        ax in -12_i16..=12,
        ay in -12_i16..=12,
        bx in -12_i16..=12,
        by in -12_i16..=12,
        dx in -12_i16..=12,
        dy in -12_i16..=12,
        az0 in -12_i16..=12,
        az1 in -12_i16..=12,
        bz0 in -12_i16..=12,
        bz1 in -12_i16..=12,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let az0 = i64::from(az0);
        let az1 = i64::from(az1);
        let bz0 = i64::from(bz0);
        let bz1 = i64::from(bz1);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", Real::from(ax), Real::from(ay), Real::from(az0));
        let b = sketch.add_point3d("b", Real::from(ax + dx), Real::from(ay + dy), Real::from(az1));
        let c = sketch.add_point3d("c", Real::from(bx), Real::from(by), Real::from(bz0));
        let d = sketch.add_point3d("d", Real::from(bx + dx), Real::from(by + dy), Real::from(bz1));
        let handle = sketch.add_projected_equal_point_point_distances3(
            "projected equal point distances",
            workplane,
            a,
            b,
            c,
            d,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedEqualPointPointDistances)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedEqualPointPointDistancesPolynomial
        );
    }

    #[test]
    fn sketch_projected_point_radius_equality_identity_workplane_matches_arc_radius(
        sx in -12_i16..=12,
        sy in -12_i16..=12,
        radius in 1_i16..=12,
        sign_x in 0_u8..=1,
        z0 in -12_i16..=12,
        z1 in -12_i16..=12,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        bad_delta in 1_i16..=4,
    ) {
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let radius = i64::from(radius);
        let dx = if sign_x == 0 { radius } else { -radius };
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let bad_delta = i64::from(bad_delta);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", Real::from(sx), Real::from(sy), Real::from(z0));
        let b = sketch.add_point3d("b", Real::from(sx + dx), Real::from(sy), Real::from(z1));
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let bad_radius = sketch.add_distance("bad radius", Real::from(radius + bad_delta));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let bad_circle = sketch.add_circle2("bad circle", center, bad_radius);
        let handle =
            sketch.add_projected_point_radius_equality3("projected point radius", workplane, a, b, arc);
        sketch.add_projected_point_radius_equality3(
            "bad projected point radius",
            workplane,
            a,
            b,
            bad_circle,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointRadiusEquality)
        );
        prop_assert_eq!(
            lowered.rows[3].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointRadiusEquality)
        );
        let unit_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let valid_zero = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(unit_zero);
        prop_assert!(valid_zero);
        prop_assert!(!certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedPointRadiusEqualityPolynomial
        );
    }

    #[test]
    fn sketch_projected_point_line_radius_equality_identity_workplane_matches_arc_radius(
        lx in -10_i16..=10,
        ly in -10_i16..=10,
        px_offset in -10_i16..=10,
        line_dx in 1_i16..=12,
        radius in 1_i16..=12,
        side in 0_u8..=1,
        z0 in -12_i16..=12,
        z1 in -12_i16..=12,
        pz in -12_i16..=12,
        bad_delta in 1_i16..=4,
    ) {
        let lx = i64::from(lx);
        let ly = i64::from(ly);
        let px_offset = i64::from(px_offset);
        let line_dx = i64::from(line_dx);
        let radius = i64::from(radius);
        let signed_radius = if side == 0 { radius } else { -radius };
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let pz = i64::from(pz);
        let bad_delta = i64::from(bad_delta);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let line_start = sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(z0));
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(lx + line_dx),
            Real::from(ly),
            Real::from(z1),
        );
        let point = sketch.add_point3d(
            "point",
            Real::from(lx + px_offset),
            Real::from(ly + signed_radius),
            Real::from(pz),
        );
        let line = sketch.add_line_segment3("line", line_start, line_end);
        let center = sketch.add_point2d("center", Real::from(0), Real::from(0));
        let start = sketch.add_point2d("start", Real::from(radius), Real::from(0));
        let end = sketch.add_point2d("end", Real::from(0), Real::from(radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let bad_radius = sketch.add_distance("bad radius", Real::from(radius + bad_delta));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let bad_circle = sketch.add_circle2("bad circle", center, bad_radius);
        let handle = sketch.add_projected_point_line_radius_equality3(
            "projected point-line radius",
            workplane,
            point,
            line,
            arc,
        );
        sketch.add_projected_point_line_radius_equality3(
            "bad projected point-line radius",
            workplane,
            point,
            line,
            bad_circle,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointLineRadiusEquality)
        );
        prop_assert_eq!(
            lowered.rows[3].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointLineRadiusEquality)
        );
        let unit_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let valid_zero = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(unit_zero);
        prop_assert!(valid_zero);
        prop_assert!(!certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedPointLineRadiusEqualityPolynomial
        );
    }

    #[test]
    fn sketch_projected_point_on_circle_rows_replay_identity_workplane_points(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -4_i16..=4,
        cy in -4_i16..=4,
        radius in 1_i16..=8,
        normal_offset in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let normal_offset = i64::from(normal_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let circle = sketch.add_circle2("circle", center, radius_entity);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + normal_offset),
        );
        let handle =
            sketch.add_projected_point_on_circle3("projected point on circle", workplane, point, circle);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 2);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(lowered.rows[1].strategy, Some(SketchResidualStrategy::ProjectedSquaredIncidence));
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::ProjectedCircleIncidencePolynomial
        );
    }

    #[test]
    fn sketch_projected_point_concentric_rows_match_identity_workplane_centers(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        radius in 1_i16..=12,
        normal_offset in -8_i16..=8,
        dx in -6_i16..=6,
        dy in -6_i16..=6,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let normal_offset = i64::from(normal_offset);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + cx),
            Real::from(oy + cy),
            Real::from(oz + normal_offset),
        );
        let handle =
            sketch.add_projected_point_concentric3("projected point concentric", workplane, point, arc);
        if dx != 0 || dy != 0 {
            let shifted_center =
                sketch.add_point2d("shifted center", Real::from(cx + dx), Real::from(cy + dy));
            let shifted = sketch.add_circle2("shifted", shifted_center, radius_entity);
            sketch.add_projected_point_concentric3(
                "bad projected point concentric",
                workplane,
                point,
                shifted,
            );
        }

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(lowered.rows[1].strategy, Some(SketchResidualStrategy::ProjectedConcentricity));
        prop_assert_eq!(lowered.rows[2].strategy, Some(SketchResidualStrategy::ProjectedConcentricity));
        let unit_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let x_zero = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let y_zero = matches!(
            certification.rows[2].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(unit_zero);
        prop_assert!(x_zero);
        prop_assert!(y_zero);
        if dx != 0 || dy != 0 {
            prop_assert!(!certification.all_satisfied());
        }
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::ProjectedConcentricCenterCoordinatePolynomial
        );
        prop_assert_eq!(
            forms.forms[2].kind,
            SketchResidualFormKind::ProjectedConcentricCenterCoordinatePolynomial
        );
    }

    #[test]
    fn sketch_projected_point_on_line_rows_replay_identity_workplane_collinearity(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        sx in -8_i16..=8,
        sy in -8_i16..=8,
        ux in -6_i16..=6,
        uy in -6_i16..=6,
        t in -8_i16..=8,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        zp in -8_i16..=8,
    ) {
        prop_assume!(ux != 0 || uy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let ux = i64::from(ux);
        let uy = i64::from(uy);
        let t = i64::from(t);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let zp = i64::from(zp);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let start =
            sketch.add_point3d("start", Real::from(ox + sx), Real::from(oy + sy), Real::from(oz + z0));
        let end = sketch.add_point3d(
            "end",
            Real::from(ox + sx + ux),
            Real::from(oy + sy + uy),
            Real::from(oz + z1),
        );
        let line = sketch.add_line_segment3("line", start, end);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + sx + t * ux),
            Real::from(oy + sy + t * uy),
            Real::from(oz + zp),
        );
        let handle = sketch.add_projected_point_on_line3("projected point on line", workplane, point, line);

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 2);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(lowered.rows[1].strategy, Some(SketchResidualStrategy::ProjectedPointLineIncidence));
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::ProjectedPointLineIncidencePolynomial
        );
        for form in &forms.forms {
            prop_assert_eq!(form.role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
        }
    }

    #[test]
    fn sketch_projected_line_circle_tangent_rows_replay_identity_workplane_lines(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -4_i16..=4,
        cy in -4_i16..=4,
        radius in 1_i16..=8,
        half_length in 1_i16..=8,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let half_length = i64::from(half_length);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let circle = sketch.add_circle2("circle", center, radius_entity);
        let start = sketch.add_point3d(
            "start",
            Real::from(ox + cx - half_length),
            Real::from(oy + cy + radius),
            Real::from(oz + z0),
        );
        let end = sketch.add_point3d(
            "end",
            Real::from(ox + cx + half_length),
            Real::from(oy + cy + radius),
            Real::from(oz + z1),
        );
        let line = sketch.add_line_segment3("line", start, end);
        let handle = sketch.add_projected_line_circle_tangent3(
            "projected line circle tangent",
            workplane,
            line,
            circle,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 2);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(lowered.rows[1].strategy, Some(SketchResidualStrategy::ProjectedLineCircleTangency));
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::ProjectedLineCircleTangencyPolynomial
        );
    }

    #[test]
    fn sketch_circle_circle_tangent_rows_match_generated_internal_external_contacts(
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        first_radius in 2_i16..=12,
        second_radius in 1_i16..=8,
        vertical in -4_i16..=4,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let first_radius = i64::from(first_radius);
        let second_radius = i64::from(second_radius);
        let vertical = i64::from(vertical);
        let mut sketch = SketchSolveProblem::new();
        let first_center = sketch.add_point2d("first center", Real::from(cx), Real::from(cy));
        let external_center = sketch.add_point2d(
            "external center",
            Real::from(cx + first_radius + second_radius),
            Real::from(cy),
        );
        let internal_offset = first_radius - second_radius;
        let internal_center = sketch.add_point2d(
            "internal center",
            Real::from(cx + internal_offset),
            Real::from(cy),
        );
        let first_radius_entity = sketch.add_distance("first radius", Real::from(first_radius));
        let second_radius_entity = sketch.add_distance("second radius", Real::from(second_radius));
        let first = sketch.add_circle2("first", first_center, first_radius_entity);
        let external = sketch.add_circle2("external", external_center, second_radius_entity);
        let internal = sketch.add_circle2("internal", internal_center, second_radius_entity);
        let external_handle = sketch.add_circle_circle_tangent2(
            "external tangent",
            first,
            external,
            SketchCircleTangencyBranch::External,
        );
        let internal_handle = sketch.add_circle_circle_tangent2(
            "internal tangent",
            first,
            internal,
            SketchCircleTangencyBranch::Internal,
        );
        if vertical != 0 {
            let bad_center = sketch.add_point2d(
                "bad center",
                Real::from(cx + first_radius + second_radius),
                Real::from(cy + vertical),
            );
            let bad = sketch.add_circle2("bad", bad_center, second_radius_entity);
            sketch.add_circle_circle_tangent2(
                "bad tangent",
                first,
                bad,
                SketchCircleTangencyBranch::External,
            );
        }

        let lowered = sketch.lower_to_problem();
        let context = context_from_problem(&lowered.problem);
        let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
        let external_forms = sketch.residual_forms_for_constraint(external_handle);
        let internal_forms = sketch.residual_forms_for_constraint(internal_handle);

        let all_circle_tangent_rows = lowered.rows[..2].iter().all(|row| {
            row.strategy == Some(SketchResidualStrategy::CircleCircleTangency)
                && row.status == SketchGeneratedRowStatus::Generated
        });
        prop_assert!(all_circle_tangent_rows);
        let external_certified = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let internal_certified = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(external_certified);
        prop_assert!(internal_certified);
        if vertical != 0 {
            let bad_violated = matches!(
                certification.rows[2].status,
                CertifiedCandidateStatus::CertifiedViolation { .. }
            );
            prop_assert!(bad_violated);
        }
        for forms in [external_forms, internal_forms] {
            prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
            prop_assert_eq!(forms.forms.len(), 1);
            prop_assert_eq!(
                forms.forms[0].kind,
                SketchResidualFormKind::CircleCircleTangencyPolynomial
            );
            prop_assert_eq!(forms.forms[0].role, SketchResidualFormRole::ExactProof);
            prop_assert_eq!(
                forms.forms[0].residual.eval_real(context.bindings()).unwrap(),
                Real::zero()
            );
        }
    }

    #[test]
    fn sketch_point_on_arc_rows_replay_generated_ccw_minor_branches(
        cx in -4_i16..=4,
        cy in -4_i16..=4,
        radius in 1_i16..=8,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let mut sketch = SketchSolveProblem::new();
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let point = sketch.add_point2d("point", Real::from(cx + radius), Real::from(cy));
        let handle = sketch.add_point_on_arc2(
            "point on arc",
            point,
            arc,
            SketchArcPointSweep::CounterClockwiseMinor,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 6);
        let arc_rows = lowered
            .rows
            .iter()
            .all(|row| row.strategy == Some(SketchResidualStrategy::PointArcIncidence));
        prop_assert!(arc_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 6);
        prop_assert_eq!(
            forms.forms[3].kind,
            SketchResidualFormKind::ArcIncidenceSweepBranchPredicate
        );
        prop_assert_eq!(
            forms.forms[5].kind,
            SketchResidualFormKind::ArcIncidencePointBranchPredicate
        );
    }

    #[test]
    fn sketch_projected_point_on_arc_rows_replay_identity_workplane_branches(
        ox in -4_i16..=4,
        oy in -4_i16..=4,
        oz in -4_i16..=4,
        cx in -4_i16..=4,
        cy in -4_i16..=4,
        radius in 1_i16..=8,
        normal_offset in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let normal_offset = i64::from(normal_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
        let normal = sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let point = sketch.add_point3d(
            "point",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + normal_offset),
        );
        let handle = sketch.add_projected_point_on_arc3(
            "projected point on arc",
            workplane,
            point,
            arc,
            SketchArcPointSweep::CounterClockwiseMinor,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows.len(), 7);
        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        let arc_rows = lowered.rows[1..]
            .iter()
            .all(|row| row.strategy == Some(SketchResidualStrategy::ProjectedPointArcIncidence));
        prop_assert!(arc_rows);
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 7);
        prop_assert_eq!(
            forms.forms[4].kind,
            SketchResidualFormKind::ProjectedArcIncidenceSweepBranchPredicate
        );
        prop_assert_eq!(
            forms.forms[6].kind,
            SketchResidualFormKind::ProjectedArcIncidencePointBranchPredicate
        );
    }

    #[test]
    fn sketch_projected_distance_range_identity_workplane_matches_horizontal_bounds(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        az in -16_i16..=16,
        u in -16_i16..=16,
        normal_offset in -16_i16..=16,
        lower_slack in 0_i16..=8,
        upper_slack in 0_i16..=8,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let az = i64::from(az);
        let u = i64::from(u);
        let normal_offset = i64::from(normal_offset);
        let distance = u.abs();
        let lower = (distance - i64::from(lower_slack)).max(0);
        let upper = distance + i64::from(upper_slack);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", Real::from(ax), Real::from(ay), Real::from(az));
        let b = sketch.add_point3d(
            "b",
            Real::from(ax + u),
            Real::from(ay),
            Real::from(az + normal_offset),
        );
        let handle = sketch.add_projected_point_point_distance_range(
            "projected clearance window",
            workplane,
            a,
            b,
            Some(Real::from(lower)),
            Some(Real::from(upper)),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::BoundedSquaredProjectedDistance)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::BoundedSquaredProjectedDistancePolynomial
        );
    }

    #[test]
    fn sketch_projected_point_line_distance_identity_workplane_ignores_normal_offsets(
        lx in -12_i16..=12,
        ly in -12_i16..=12,
        lz in -12_i16..=12,
        length in 1_i16..=12,
        distance in 0_i16..=12,
        point_z_offset in -12_i16..=12,
        line_z_offset in -12_i16..=12,
    ) {
        let lx = i64::from(lx);
        let ly = i64::from(ly);
        let lz = i64::from(lz);
        let length = i64::from(length);
        let distance = i64::from(distance);
        let point_z_offset = i64::from(point_z_offset);
        let line_z_offset = i64::from(line_z_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let line_start = sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(lz));
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(lx + length),
            Real::from(ly),
            Real::from(lz + line_z_offset),
        );
        let line = sketch.add_line_segment3("line", line_start, line_end);
        let point = sketch.add_point3d(
            "point",
            Real::from(lx + length / 2),
            Real::from(ly + distance),
            Real::from(lz + point_z_offset),
        );
        let distance_entity = sketch.add_distance("distance", Real::from(distance));
        let handle = sketch.add_projected_point_line_distance(
            "projected point-line",
            workplane,
            point,
            line,
            distance_entity,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointLineDistance)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 4);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedPointLineDistancePolynomial
        );
    }

    #[test]
    fn sketch_projected_point_line_distance_range_identity_workplane_matches_horizontal_bounds(
        lx in -12_i16..=12,
        ly in -12_i16..=12,
        lz in -12_i16..=12,
        length in 1_i16..=12,
        distance in 0_i16..=12,
        lower_slack in 0_i16..=8,
        upper_slack in 0_i16..=8,
        point_z_offset in -12_i16..=12,
        line_z_offset in -12_i16..=12,
    ) {
        let lx = i64::from(lx);
        let ly = i64::from(ly);
        let lz = i64::from(lz);
        let length = i64::from(length);
        let distance = i64::from(distance);
        let lower = (distance - i64::from(lower_slack)).max(0);
        let upper = distance + i64::from(upper_slack);
        let point_z_offset = i64::from(point_z_offset);
        let line_z_offset = i64::from(line_z_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let line_start = sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(lz));
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(lx + length),
            Real::from(ly),
            Real::from(lz + line_z_offset),
        );
        let line = sketch.add_line_segment3("line", line_start, line_end);
        let point = sketch.add_point3d(
            "point",
            Real::from(lx + length / 2),
            Real::from(ly + distance),
            Real::from(lz + point_z_offset),
        );
        let handle = sketch.add_projected_point_line_distance_range(
            "projected point-line clearance window",
            workplane,
            point,
            line,
            Some(Real::from(lower)),
            Some(Real::from(upper)),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::BoundedSquaredProjectedPointLineDistancePolynomial
        );
    }

    #[test]
    fn sketch_projected_equal_length_lines3_identity_workplane_ignores_normal_offsets(
        ax in -12_i16..=12,
        ay in -12_i16..=12,
        bx in -12_i16..=12,
        by in -12_i16..=12,
        dx in -12_i16..=12,
        dy in -12_i16..=12,
        az0 in -12_i16..=12,
        az1 in -12_i16..=12,
        bz0 in -12_i16..=12,
        bz1 in -12_i16..=12,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let az0 = i64::from(az0);
        let az1 = i64::from(az1);
        let bz0 = i64::from(bz0);
        let bz1 = i64::from(bz1);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a0 = sketch.add_point3d("a0", Real::from(ax), Real::from(ay), Real::from(az0));
        let a1 = sketch.add_point3d("a1", Real::from(ax + dx), Real::from(ay + dy), Real::from(az1));
        let b0 = sketch.add_point3d("b0", Real::from(bx), Real::from(by), Real::from(bz0));
        let b1 = sketch.add_point3d("b1", Real::from(bx + dx), Real::from(by + dy), Real::from(bz1));
        let a = sketch.add_line_segment3("a", a0, a1);
        let b = sketch.add_line_segment3("b", b0, b1);
        let handle = sketch.add_projected_equal_length_lines3("projected equal length", workplane, a, b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineLengthEquality)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedLineLengthEqualityPolynomial
        );
    }

    #[test]
    fn sketch_projected_line_radius_equality_identity_workplane_matches_arc_radius(
        sx in -12_i16..=12,
        sy in -12_i16..=12,
        radius in 1_i16..=12,
        sign in 0_u8..=1,
        z0 in -12_i16..=12,
        z1 in -12_i16..=12,
        cx in -8_i16..=8,
        cy in -8_i16..=8,
        bad_delta in 1_i16..=4,
    ) {
        let sx = i64::from(sx);
        let sy = i64::from(sy);
        let radius = i64::from(radius);
        let ux = if sign == 0 { radius } else { -radius };
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let bad_delta = i64::from(bad_delta);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let start3 = sketch.add_point3d("start3", Real::from(sx), Real::from(sy), Real::from(z0));
        let end3 = sketch.add_point3d("end3", Real::from(sx + ux), Real::from(sy), Real::from(z1));
        let line = sketch.add_line_segment3("line", start3, end3);
        let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
        let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
        let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
        let radius_entity = sketch.add_distance("radius", Real::from(radius));
        let bad_radius = sketch.add_distance("bad radius", Real::from(radius + bad_delta));
        let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
        let bad_circle = sketch.add_circle2("bad circle", center, bad_radius);
        let handle =
            sketch.add_projected_line_radius_equality3("projected line radius", workplane, line, arc);
        sketch.add_projected_line_radius_equality3(
            "bad projected line radius",
            workplane,
            line,
            bad_circle,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.rows[0].strategy, Some(SketchResidualStrategy::WorkplaneUnitQuaternion));
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineRadiusEquality)
        );
        prop_assert_eq!(
            lowered.rows[3].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineRadiusEquality)
        );
        let unit_zero = matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        let valid_zero = matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        );
        prop_assert!(unit_zero);
        prop_assert!(valid_zero);
        prop_assert!(!certification.all_satisfied());
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedLineRadiusEqualityPolynomial
        );
    }

    #[test]
    fn sketch_projected_line_length_range_identity_workplane_matches_horizontal_bounds(
        lx in -12_i16..=12,
        ly in -12_i16..=12,
        lz in -12_i16..=12,
        length in 1_i16..=12,
        lower_slack in 0_i16..=8,
        upper_slack in 0_i16..=8,
        line_z_offset in -12_i16..=12,
    ) {
        let lx = i64::from(lx);
        let ly = i64::from(ly);
        let lz = i64::from(lz);
        let length = i64::from(length);
        let lower = (length - i64::from(lower_slack)).max(0);
        let upper = length + i64::from(upper_slack);
        let line_z_offset = i64::from(line_z_offset);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let line_start = sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(lz));
        let line_end = sketch.add_point3d(
            "line end",
            Real::from(lx + length),
            Real::from(ly),
            Real::from(lz + line_z_offset),
        );
        let line = sketch.add_line_segment3("line", line_start, line_end);
        let handle = sketch.add_projected_line_length_range3(
            "projected line length window",
            workplane,
            line,
            Some(Real::from(lower)),
            Some(Real::from(upper)),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::BoundedSquaredProjectedLineLength)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::BoundedSquaredProjectedLineLengthPolynomial
        );
    }

    #[test]
    fn sketch_projected_length_ratio_lines3_identity_workplane_matches_scaled_directions(
        ax in -8_i16..=8,
        ay in -8_i16..=8,
        bx in -8_i16..=8,
        by in -8_i16..=8,
        dx in -6_i16..=6,
        dy in -6_i16..=6,
        scale in 1_i16..=4,
        az0 in -8_i16..=8,
        az1 in -8_i16..=8,
        bz0 in -8_i16..=8,
        bz1 in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let scale = i64::from(scale);
        let az0 = i64::from(az0);
        let az1 = i64::from(az1);
        let bz0 = i64::from(bz0);
        let bz1 = i64::from(bz1);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a0 = sketch.add_point3d("a0", Real::from(ax), Real::from(ay), Real::from(az0));
        let a1 = sketch.add_point3d(
            "a1",
            Real::from(ax + scale * dx),
            Real::from(ay + scale * dy),
            Real::from(az1),
        );
        let b0 = sketch.add_point3d("b0", Real::from(bx), Real::from(by), Real::from(bz0));
        let b1 = sketch.add_point3d("b1", Real::from(bx + dx), Real::from(by + dy), Real::from(bz1));
        let a = sketch.add_line_segment3("a", a0, a1);
        let b = sketch.add_line_segment3("b", b0, b1);
        let handle = sketch.add_projected_length_ratio_lines3(
            "projected length ratio",
            workplane,
            a,
            b,
            Real::from(scale),
            Real::from(1),
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedLineLengthRatioPolynomial
        );
    }

    #[test]
    fn sketch_projected_length_difference_lines3_identity_workplane_matches_horizontal_lengths(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        shorter in 0_i16..=12,
        difference in 0_i16..=12,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let shorter = i64::from(shorter);
        let difference = i64::from(difference);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let longer = shorter + difference;
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let long0 = sketch.add_point3d("long0", Real::from(x0), Real::from(y0), Real::from(z0));
        let long1 = sketch.add_point3d("long1", Real::from(x0 + longer), Real::from(y0), Real::from(z1));
        let short0 = sketch.add_point3d("short0", Real::from(x0), Real::from(y0 + 4), Real::from(z2));
        let short1 = sketch.add_point3d("short1", Real::from(x0 + shorter), Real::from(y0 + 4), Real::from(z3));
        let longer = sketch.add_line_segment3("longer", long0, long1);
        let shorter = sketch.add_line_segment3("shorter", short0, short1);
        let difference = sketch.add_distance("difference", Real::from(difference));
        let handle = sketch.add_projected_length_difference_lines3(
            "projected length difference",
            workplane,
            longer,
            shorter,
            difference,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineLengthDifference)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedLineLengthDifferencePolynomial
        );
    }

    #[test]
    fn sketch_projected_point_distance_difference3_identity_workplane_matches_horizontal_lengths(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        shorter in 0_i16..=12,
        difference in 0_i16..=12,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let shorter = i64::from(shorter);
        let difference = i64::from(difference);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let longer = shorter + difference;
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let long0 = sketch.add_point3d("long0", Real::from(x0), Real::from(y0), Real::from(z0));
        let long1 = sketch.add_point3d("long1", Real::from(x0 + longer), Real::from(y0), Real::from(z1));
        let short0 = sketch.add_point3d("short0", Real::from(x0), Real::from(y0 + 4), Real::from(z2));
        let short1 = sketch.add_point3d("short1", Real::from(x0 + shorter), Real::from(y0 + 4), Real::from(z3));
        let difference = sketch.add_distance("difference", Real::from(difference));
        let handle = sketch.add_projected_point_distance_difference3(
            "projected point distance difference",
            workplane,
            (long0, long1),
            (short0, short1),
            difference,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointDistanceDifference)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedPointDistanceDifferencePolynomial
        );
        prop_assert_eq!(
            forms.forms[2].kind,
            SketchResidualFormKind::ProjectedPointDistanceDifferenceBranchPredicate
        );
    }

    #[test]
    fn sketch_projected_equal_length_point_line_distance3_identity_workplane_matches_horizontal_distance(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        length in 0_i16..=12,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        z4 in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let length = i64::from(length);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let z4 = i64::from(z4);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let len0 = sketch.add_point3d("len0", Real::from(x0), Real::from(y0 + 9), Real::from(z0));
        let len1 = sketch.add_point3d("len1", Real::from(x0 + length), Real::from(y0 + 9), Real::from(z1));
        let point = sketch.add_point3d("point", Real::from(x0), Real::from(y0 + length), Real::from(z2));
        let dist0 = sketch.add_point3d("dist0", Real::from(x0 - 5), Real::from(y0), Real::from(z3));
        let dist1 = sketch.add_point3d("dist1", Real::from(x0 + 5), Real::from(y0), Real::from(z4));
        let length_line = sketch.add_line_segment3("length_line", len0, len1);
        let distance_line = sketch.add_line_segment3("distance_line", dist0, dist1);
        let handle = sketch.add_projected_equal_length_point_line_distance3(
            "projected length equals point-line distance",
            workplane,
            length_line,
            point,
            distance_line,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedLineLengthPointLineDistancePolynomial
        );
    }

    #[test]
    fn sketch_projected_equal_point_distance_point_line_distance3_identity_workplane_matches_horizontal_distance(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        distance in 0_i16..=12,
        sign in 0_u8..=1,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        z4 in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let distance = i64::from(distance);
        let signed_distance = if sign == 0 { distance } else { -distance };
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let z4 = i64::from(z4);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", Real::from(x0), Real::from(y0 + 9), Real::from(z0));
        let b = sketch.add_point3d("b", Real::from(x0 + distance), Real::from(y0 + 9), Real::from(z1));
        let point = sketch.add_point3d("point", Real::from(x0), Real::from(y0 + signed_distance), Real::from(z2));
        let dist0 = sketch.add_point3d("dist0", Real::from(x0 - 5), Real::from(y0), Real::from(z3));
        let dist1 = sketch.add_point3d("dist1", Real::from(x0 + 5), Real::from(y0), Real::from(z4));
        let distance_line = sketch.add_line_segment3("distance_line", dist0, dist1);
        let handle = sketch.add_projected_equal_point_distance_point_line_distance3(
            "projected point distance equals point-line distance",
            workplane,
            a,
            b,
            point,
            distance_line,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointDistancePointLineDistance)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedPointDistancePointLineDistancePolynomial
        );
    }

    #[test]
    fn sketch_projected_equal_point_line_distances3_identity_workplane_matches_horizontal_offsets(
        x0 in -8_i16..=8,
        y0 in -8_i16..=8,
        distance in 0_i16..=12,
        z0 in -8_i16..=8,
        z1 in -8_i16..=8,
        z2 in -8_i16..=8,
        z3 in -8_i16..=8,
        z4 in -8_i16..=8,
        z5 in -8_i16..=8,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let distance = i64::from(distance);
        let z0 = i64::from(z0);
        let z1 = i64::from(z1);
        let z2 = i64::from(z2);
        let z3 = i64::from(z3);
        let z4 = i64::from(z4);
        let z5 = i64::from(z5);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a_point = sketch.add_point3d("a_point", Real::from(x0), Real::from(y0 + distance), Real::from(z0));
        let a0 = sketch.add_point3d("a0", Real::from(x0 - 5), Real::from(y0), Real::from(z1));
        let a1 = sketch.add_point3d("a1", Real::from(x0 + 5), Real::from(y0), Real::from(z2));
        let b_point = sketch.add_point3d("b_point", Real::from(x0 + 2), Real::from(y0 + 20 + distance), Real::from(z3));
        let b0 = sketch.add_point3d("b0", Real::from(x0 - 7), Real::from(y0 + 20), Real::from(z4));
        let b1 = sketch.add_point3d("b1", Real::from(x0 + 7), Real::from(y0 + 20), Real::from(z5));
        let a_line = sketch.add_line_segment3("a_line", a0, a1);
        let b_line = sketch.add_line_segment3("b_line", b0, b1);
        let handle = sketch.add_projected_equal_point_line_distances3(
            "projected equal point-line distances",
            workplane,
            a_point,
            a_line,
            b_point,
            b_line,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 2);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::SquaredProjectedEqualPointLineDistances)
        );
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 2);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::SquaredProjectedEqualPointLineDistancesPolynomial
        );
    }

    #[test]
    fn sketch_projected_oriented_angle_identity_workplane_matches_scaled_directions(
        ox in -8_i16..=8,
        oy in -8_i16..=8,
        oz in -8_i16..=8,
        ax in 1_i16..=12,
        by in 1_i16..=12,
        scale_a in 1_i16..=8,
        scale_b in 1_i16..=8,
        az in -8_i16..=8,
        bz in -8_i16..=8,
    ) {
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let ax = i64::from(ax);
        let by = i64::from(by);
        let scale_a = i64::from(scale_a);
        let scale_b = i64::from(scale_b);
        let az = i64::from(az);
        let bz = i64::from(bz);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let base = sketch.add_point3d("base", Real::from(ox), Real::from(oy), Real::from(oz));
        let a_end = sketch.add_point3d("a end", Real::from(ox + ax), Real::from(oy), Real::from(oz + az));
        let b_end = sketch.add_point3d("b end", Real::from(ox), Real::from(oy + by), Real::from(oz + bz));
        let c_end = sketch.add_point3d("c end", Real::from(ox + scale_a * ax), Real::from(oy), Real::from(oz - az));
        let d_end = sketch.add_point3d("d end", Real::from(ox), Real::from(oy + scale_b * by), Real::from(oz - bz));
        let a = sketch.add_line_segment3("a", base, a_end);
        let b = sketch.add_line_segment3("b", base, b_end);
        let c = sketch.add_line_segment3("c", base, c_end);
        let d = sketch.add_line_segment3("d", base, d_end);
        let handle = sketch.add_projected_equal_oriented_angle_lines3(
            "projected oriented angle",
            workplane,
            a,
            b,
            c,
            d,
        );

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let forms = sketch.residual_forms_for_constraint(handle);

        prop_assert_eq!(lowered.problem.constraints.len(), 3);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
        );
        prop_assert!(lowered.rows[1..]
            .iter()
            .all(|row| row.strategy
                == Some(SketchResidualStrategy::ProjectedOrientedAngleEquality)));
        prop_assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(forms.forms.len(), 3);
        prop_assert_eq!(
            forms.forms[1].kind,
            SketchResidualFormKind::ProjectedOrientedAngleVectorCollinearityPolynomial
        );
        prop_assert_eq!(
            forms.forms[2].kind,
            SketchResidualFormKind::ProjectedOrientedAngleSameBranchPredicate
        );
    }

    #[test]
    fn sketch_projected_line_orientation_identity_workplane_matches_direction_predicates(
        ox in -8_i16..=8,
        oy in -8_i16..=8,
        oz in -8_i16..=8,
        dx in -8_i16..=8,
        dy in -8_i16..=8,
        parallel_scale in 1_i16..=4,
        perpendicular_scale in 1_i16..=4,
        a_z in -8_i16..=8,
        b_z in -8_i16..=8,
        c_z in -8_i16..=8,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let ox = i64::from(ox);
        let oy = i64::from(oy);
        let oz = i64::from(oz);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let parallel_scale = i64::from(parallel_scale);
        let perpendicular_scale = i64::from(perpendicular_scale);
        let a_z = i64::from(a_z);
        let b_z = i64::from(b_z);
        let c_z = i64::from(c_z);
        let mut sketch = SketchSolveProblem::new();
        let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
        let normal = sketch.add_normal3d("normal", Real::from(1), Real::from(0), Real::from(0), Real::from(0));
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let base = sketch.add_point3d("base", Real::from(ox), Real::from(oy), Real::from(oz));
        let a_end = sketch.add_point3d("a end", Real::from(ox + dx), Real::from(oy + dy), Real::from(oz + a_z));
        let b_end = sketch.add_point3d(
            "b end",
            Real::from(ox + parallel_scale * dx),
            Real::from(oy + parallel_scale * dy),
            Real::from(oz + b_z),
        );
        let c_end = sketch.add_point3d(
            "c end",
            Real::from(ox - perpendicular_scale * dy),
            Real::from(oy + perpendicular_scale * dx),
            Real::from(oz + c_z),
        );
        let a = sketch.add_line_segment3("a", base, a_end);
        let b = sketch.add_line_segment3("b", base, b_end);
        let c = sketch.add_line_segment3("c", base, c_end);
        let parallel = sketch.add_projected_parallel_lines3("projected parallel", workplane, a, b);
        let perpendicular =
            sketch.add_projected_perpendicular_lines3("projected perpendicular", workplane, a, c);
        let same_direction =
            sketch.add_projected_same_direction_lines3("projected same direction", workplane, a, b);

        let lowered = sketch.lower_to_problem();
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        let parallel_forms = sketch.residual_forms_for_constraint(parallel);
        let perpendicular_forms = sketch.residual_forms_for_constraint(perpendicular);
        let same_direction_forms = sketch.residual_forms_for_constraint(same_direction);

        prop_assert_eq!(lowered.problem.constraints.len(), 7);
        prop_assert!(lowered.all_generated());
        prop_assert!(certification.all_satisfied());
        prop_assert_eq!(
            lowered.rows[1].strategy,
            Some(SketchResidualStrategy::ProjectedDirectionCrossProduct)
        );
        prop_assert_eq!(
            lowered.rows[3].strategy,
            Some(SketchResidualStrategy::ProjectedDirectionDotProduct)
        );
        prop_assert_eq!(
            lowered.rows[5].strategy,
            Some(SketchResidualStrategy::ProjectedDirectionSameOrientation)
        );
        prop_assert_eq!(
            lowered.rows[6].strategy,
            Some(SketchResidualStrategy::ProjectedDirectionSameOrientation)
        );
        prop_assert_eq!(parallel_forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(
            parallel_forms.forms[1].kind,
            SketchResidualFormKind::ProjectedDirectionCrossProductPolynomial
        );
        prop_assert_eq!(perpendicular_forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(
            perpendicular_forms.forms[1].kind,
            SketchResidualFormKind::ProjectedDirectionDotProductPolynomial
        );
        prop_assert_eq!(same_direction_forms.status, SketchResidualFormsStatus::Generated);
        prop_assert_eq!(same_direction_forms.forms.len(), 3);
        prop_assert_eq!(
            same_direction_forms.forms[2].kind,
            SketchResidualFormKind::ProjectedDirectionSameOrientationPredicate
        );
    }

    #[test]
    fn sketch_entity_domain_generated_tangent_lengths_match_integer_points(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        bx in -16_i16..=16,
        by in -16_i16..=16,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let bx = i64::from(bx);
        let by = i64::from(by);
        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
        let b = sketch.add_point2d("b", Real::from(bx), Real::from(by));
        let tangent = sketch.add_line_segment2("tangent", a, b);
        prop_assert!(sketch.add_entity_domain(
            tangent,
            SketchEntityDomain::NonzeroTangentLineSegment2,
        ));

        let report = preflight_sketch_entity_domains(&sketch);
        let row = report
            .checks
            .iter()
            .find(|check| {
                check.entity == tangent
                    && check.kind == SketchEntityDomainKind::NonzeroTangentLineSegment2
            })
            .expect("tangent domain row should be emitted");
        let expected_nonzero = ax != bx || ay != by;

        prop_assert_eq!(
            row.status == SketchEntityDomainStatus::CertifiedValid,
            expected_nonzero
        );
        prop_assert_eq!(
            row.status == SketchEntityDomainStatus::CertifiedInvalid,
            !expected_nonzero
        );
    }

    #[test]
    fn failed_constraint_diagnostics_generated_duplicate_affine_rows_are_redundant(
        value in -32_i16..=32,
    ) {
        let value = i64::from(value);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(value));
        problem.add_constraint(Constraint::equality(
            "x row",
            x.clone() - Expr::int(value),
        ));
        problem.add_constraint(Constraint::equality(
            "duplicate row",
            x * Expr::int(2) - Expr::int(value * 2),
        ));

        let report = diagnose_failed_constraints(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
        );

        prop_assert_eq!(report.blocking_rows, 0);
        prop_assert_eq!(report.rank_redundant_rows, 2);
        prop_assert_eq!(report.rows.len(), 2);
        prop_assert!(report
            .rows
            .iter()
            .all(|row| row.status == FailedConstraintStatus::RankRedundant));
    }

    #[test]
    fn sketch_failed_constraint_report_generated_distance_misses_source_constraint(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        dx in -16_i16..=16,
        dy in -16_i16..=16,
        wrong_distance in 0_i16..=32,
    ) {
        let ax = i64::from(ax);
        let ay = i64::from(ay);
        let dx = i64::from(dx);
        let dy = i64::from(dy);
        let squared = dx * dx + dy * dy;
        let wrong_distance = i64::from(wrong_distance);
        prop_assume!(squared != wrong_distance * wrong_distance);

        let mut sketch = SketchSolveProblem::new();
        let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
        let b = sketch.add_point2d("b", Real::from(ax + dx), Real::from(ay + dy));
        let distance = sketch.add_distance("wrong distance", Real::from(wrong_distance));
        let handle = sketch.add_point_point_distance("generated sketch miss", a, b, distance);

        let report = diagnose_sketch_failed_constraints(&sketch);

        prop_assert!(report.has_blocking_rows());
        prop_assert_eq!(report.blocking_rows, 1);
        prop_assert_eq!(report.exact_failure_rows, 1);
        prop_assert_eq!(report.lowering_failure_rows, 0);
        prop_assert_eq!(report.rows.len(), 1);
        prop_assert_eq!(report.rows[0].constraint, handle);
        prop_assert!(matches!(
            report.rows[0].status,
            SketchFailedConstraintStatus::ExactFailure(
                FailedConstraintStatus::CertifiedCandidateViolation
            )
        ));
    }

    #[test]
    fn failed_constraint_single_removal_generated_single_miss_clears(
        value in -32_i16..=32,
        target in -32_i16..=32,
    ) {
        prop_assume!(value != target);
        let value = i64::from(value);
        let target = i64::from(target);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(value));
        problem.add_constraint(Constraint::equality(
            "generated miss",
            x - Expr::int(target),
        ));

        let report = search_failed_constraint_single_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
        );

        prop_assert_eq!(report.original.blocking_rows, 1);
        prop_assert_eq!(report.probes.len(), 1);
        prop_assert_eq!(report.clearing_single_removals, 1);
        prop_assert_eq!(
            &report.probes[0].removal_status,
            &FailedConstraintRemovalStatus::ClearsAllBlockingRows
        );
    }

    #[test]
    fn failed_constraint_pair_removal_generated_two_misses_clear(
        value in -32_i16..=32,
        first_target in -32_i16..=32,
        second_target in -32_i16..=32,
    ) {
        prop_assume!(value != first_target);
        prop_assume!(value != second_target);
        let value = i64::from(value);
        let first_target = i64::from(first_target);
        let second_target = i64::from(second_target);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(value));
        problem.add_constraint(Constraint::equality(
            "first generated miss",
            x.clone() - Expr::int(first_target),
        ));
        problem.add_constraint(Constraint::equality(
            "second generated miss",
            x - Expr::int(second_target),
        ));

        let report = search_failed_constraint_pair_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
        );

        prop_assert_eq!(report.original.blocking_rows, 2);
        prop_assert_eq!(report.probes.len(), 1);
        prop_assert_eq!(report.clearing_pair_removals, 1);
        prop_assert_eq!(
            &report.probes[0].removal_status,
            &FailedConstraintRemovalStatus::ClearsAllBlockingRows
        );
    }

    #[test]
    fn failed_constraint_set_removal_generated_three_misses_clear_at_cardinality_three(
        value in -32_i16..=32,
        first_target in -32_i16..=32,
        second_target in -32_i16..=32,
        third_target in -32_i16..=32,
    ) {
        prop_assume!(value != first_target);
        prop_assume!(value != second_target);
        prop_assume!(value != third_target);
        let value = i64::from(value);
        let first_target = i64::from(first_target);
        let second_target = i64::from(second_target);
        let third_target = i64::from(third_target);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(value));
        problem.add_constraint(Constraint::equality(
            "first generated miss",
            x.clone() - Expr::int(first_target),
        ));
        problem.add_constraint(Constraint::equality(
            "second generated miss",
            x.clone() - Expr::int(second_target),
        ));
        problem.add_constraint(Constraint::equality(
            "third generated miss",
            x - Expr::int(third_target),
        ));

        let pair_bounded = search_failed_constraint_set_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            2,
        );
        prop_assert_eq!(pair_bounded.original.blocking_rows, 3);
        prop_assert_eq!(pair_bounded.probes.len(), 6);
        prop_assert_eq!(pair_bounded.clearing_removals, 0);

        let triple_bounded = search_failed_constraint_set_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            3,
        );
        prop_assert_eq!(triple_bounded.original.blocking_rows, 3);
        prop_assert_eq!(triple_bounded.probes.len(), 7);
        prop_assert_eq!(triple_bounded.clearing_removals, 1);
        prop_assert!(triple_bounded.has_removal_resolution());
    }

    #[test]
    fn failed_constraint_minimal_removal_generated_three_misses_stop_at_cardinality_three(
        value in -32_i16..=32,
        first_target in -32_i16..=32,
        second_target in -32_i16..=32,
        third_target in -32_i16..=32,
    ) {
        prop_assume!(value != first_target);
        prop_assume!(value != second_target);
        prop_assume!(value != third_target);
        let value = i64::from(value);
        let first_target = i64::from(first_target);
        let second_target = i64::from(second_target);
        let third_target = i64::from(third_target);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(value));
        problem.add_constraint(Constraint::equality(
            "first generated miss",
            x.clone() - Expr::int(first_target),
        ));
        problem.add_constraint(Constraint::equality(
            "second generated miss",
            x.clone() - Expr::int(second_target),
        ));
        problem.add_constraint(Constraint::equality(
            "third generated miss",
            x - Expr::int(third_target),
        ));

        let pair_bound = search_failed_constraint_minimal_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            2,
        );
        prop_assert_eq!(pair_bound.original.blocking_rows, 3);
        prop_assert_eq!(pair_bound.minimal_cardinality, None);
        prop_assert_eq!(pair_bound.probes.len(), 6);
        prop_assert_eq!(pair_bound.clearing_removals, 0);

        let triple_bound = search_failed_constraint_minimal_removals(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            3,
        );
        prop_assert_eq!(triple_bound.original.blocking_rows, 3);
        prop_assert_eq!(triple_bound.minimal_cardinality, Some(3));
        prop_assert_eq!(triple_bound.probes.len(), 1);
        prop_assert_eq!(triple_bound.clearing_removals, 1);
        prop_assert!(triple_bound.has_minimal_removal_resolution());
    }

    #[test]
    fn solver_block_counts_generated_row_families(
        constants in prop::collection::vec(-8_i16..=8, 1..12),
        affine_count in 0_usize..8,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", hyperreal::Real::from(1));
        for (index, constant) in constants.iter().enumerate() {
            problem.add_constraint(Constraint::equality(
                format!("constant {index}"),
                Expr::int(i64::from(*constant)),
            ));
        }
        for index in 0..affine_count {
            problem.add_constraint(Constraint::equality(
                format!("affine {index}"),
                x.clone() + Expr::int(index as i64),
            ));
        }

        let prepared = PreparedProblem::new(&problem);
        let block = PreparedSolverBlock::new(&prepared);
        let expected_contradictions = constants.iter().filter(|value| **value != 0).count();

        prop_assert_eq!(block.facts().constant_row_count, constants.len());
        prop_assert_eq!(block.facts().constant_contradiction_count, expected_contradictions);
        prop_assert_eq!(block.facts().prepared_affine_row_count, affine_count);
        prop_assert!(block.rows().iter().take(constants.len()).all(|row| matches!(
            row.kind,
            SolverBlockRowKind::ConstantCertifiedZero
                | SolverBlockRowKind::ConstantCertifiedContradiction
        )));
    }

    #[test]
    fn lossy_adapter_only_generated_reports_cover_active_rows(
        active_flags in prop::collection::vec(any::<bool>(), 1..24),
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(1));
        for (index, active) in active_flags.iter().copied().enumerate() {
            let mut constraint = Constraint::equality(
                format!("generated row {index}"),
                x.clone() - Expr::int(index as i64),
            );
            constraint.active = active;
            problem.add_constraint(constraint);
        }
        let prepared = PreparedProblem::new(&problem);
        let report = report_lossy_adapter_only_candidate(
            &prepared,
            ProposalEngineReport {
                requested: ProposalEngineKind::DampedLeastSquares,
                used: Some(ProposalEngineKind::DampedLeastSquares),
                precision: ProposalEnginePrecision::LossyF64,
                supported: true,
            },
        );
        let active_count = active_flags.iter().filter(|active| **active).count();

        prop_assert_eq!(report.rows.len(), active_count);
        prop_assert_eq!(report.lossy_adapter_only_rows, active_count);
        prop_assert_eq!(report.bounded_unknown_rows, active_count);
        prop_assert_eq!(report.certified_satisfied_rows, 0);
        prop_assert_eq!(report.certified_violation_rows, 0);
        let all_lossy_only = report.rows.iter().all(|row| matches!(
            row.status,
            CertifiedCandidateStatus::LossyAdapterOnly { .. }
        ));
        prop_assert!(all_lossy_only);
    }

    #[test]
    fn levenberg_marquardt_generated_affine_targets_report_lossy_route(
        target in (-24_i16..=24).prop_filter("nonzero target exercises dense step", |value| *value != 0),
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated affine target",
            x - Expr::int(i64::from(target)),
        ));
        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 8,
                proposal_engine: ProposalEngineKind::LevenbergMarquardt,
                ..SolverConfig::default()
            },
        });

        prop_assert!(report.proposal_engine.supported);
        prop_assert_eq!(
            report.proposal_engine.requested,
            ProposalEngineKind::LevenbergMarquardt
        );
        prop_assert_eq!(
            report.proposal_engine.used,
            Some(ProposalEngineKind::LevenbergMarquardt)
        );
        prop_assert_eq!(
            report.proposal_engine.precision,
            ProposalEnginePrecision::LossyF64
        );
        prop_assert!(!report.linear_reports.is_empty());
        prop_assert!(report.linear_reports.iter().all(|row| row.is_lossy()));
    }

    #[test]
    fn modified_newton_generated_preprocessing_counts_exact_rows(
        offset in -12_i16..=12,
        target in -12_i16..=12,
        root in -8_i16..=8,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let z = Expr::symbol(SymbolId(2), "z");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_variable("y", Real::from(0));
        problem.add_variable("z", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated substitution",
            x - y.clone() - Expr::int(i64::from(offset)),
        ));
        problem.add_constraint(Constraint::equality(
            "generated soluble affine",
            y - Expr::int(i64::from(target)),
        ));
        let root = i64::from(root);
        problem.add_constraint(Constraint::equality(
            "generated soluble quadratic",
            z.clone() * z - Expr::int(root * root),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 0,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::MaxIterations);
        prop_assert!(report.preprocessing.requested);
        prop_assert!(report.preprocessing.completed);
        prop_assert_eq!(report.preprocessing.equality_substitutions, 1);
        prop_assert_eq!(report.preprocessing.substitution_seed_classes, 1);
        prop_assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
        prop_assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
        prop_assert_eq!(report.preprocessing.affine_soluble_alone_rows, 1);
        prop_assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
        prop_assert_eq!(report.preprocessing.affine_seed_assignments, 1);
        prop_assert_eq!(report.preprocessing.rejected_affine_seed_assignments, 0);
        if root == 0 {
            prop_assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
            prop_assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
        } else {
            prop_assert_eq!(report.preprocessing.quadratic_seed_assignments, 0);
            prop_assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 1);
        }
        prop_assert_eq!(report.preprocessing.dragged_parameter_weights, 0);
        prop_assert_eq!(report.preprocessing.invalid_dragged_parameter_weights, 0);
    }

    #[test]
    fn modified_newton_generated_substitution_classes_seed_from_affine_anchor(
        start_x in -12_i16..=12,
        start_y in -12_i16..=12,
        offset in -12_i16..=12,
        target in -12_i16..=12,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(start_x)));
        problem.add_variable("y", Real::from(i64::from(start_y)));
        problem.add_constraint(Constraint::equality(
            "generated substitution",
            x - y.clone() - Expr::int(i64::from(offset)),
        ));
        problem.add_constraint(Constraint::equality(
            "generated soluble affine",
            y - Expr::int(i64::from(target)),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert_eq!(report.iterations, 0);
        prop_assert_eq!(report.preprocessing.equality_substitutions, 1);
        prop_assert_eq!(report.preprocessing.affine_seed_assignments, 1);
        prop_assert_eq!(report.preprocessing.substitution_seed_classes, 1);
        prop_assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
        prop_assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
    }

    #[test]
    fn modified_newton_generated_unanchored_substitution_seeds_from_bounds(
        start_x in -24_i16..=24,
        start_y in -24_i16..=24,
        offset in -12_i16..=12,
        target_y in -12_i16..=12,
    ) {
        let offset = i64::from(offset);
        let target_y = i64::from(target_y);
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        let x_id = problem.add_variable("x", Real::from(i64::from(start_x)));
        problem.variables[x_id.0 as usize].upper = Some(Real::from(target_y + offset));
        let y_id = problem.add_variable("y", Real::from(i64::from(start_y)));
        problem.variables[y_id.0 as usize].lower = Some(Real::from(target_y));
        problem.add_constraint(Constraint::equality(
            "generated bounded substitution",
            x - y - Expr::int(offset),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert_eq!(report.iterations, 0);
        prop_assert_eq!(report.preprocessing.equality_substitutions, 1);
        prop_assert_eq!(report.preprocessing.affine_seed_assignments, 0);
        prop_assert_eq!(report.preprocessing.substitution_seed_classes, 1);
        prop_assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
        prop_assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
    }

    #[test]
    fn modified_newton_generated_affine_seeds_converge_without_iteration(
        start in -12_i16..=12,
        target in -12_i16..=12,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(start)));
        problem.add_constraint(Constraint::equality(
            "generated soluble affine",
            x - Expr::int(i64::from(target)),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert_eq!(report.iterations, 0);
        prop_assert_eq!(report.preprocessing.affine_soluble_alone_rows, 1);
        prop_assert_eq!(report.preprocessing.affine_seed_assignments, 1);
        prop_assert_eq!(report.preprocessing.rejected_affine_seed_assignments, 0);
    }

    #[test]
    fn modified_newton_generated_unique_quadratic_seeds_converge_without_iteration(
        start in -12_i16..=12,
        root in -8_i16..=8,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let root = i64::from(root);
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(start)));
        problem.add_constraint(Constraint::equality(
            "generated double root",
            x.clone() * x.clone() - Expr::int(2 * root) * x + Expr::int(root * root),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert_eq!(report.iterations, 0);
        prop_assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
        prop_assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
        prop_assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
    }

    #[test]
    fn modified_newton_generated_bounded_quadratic_seeds_choose_exact_branch(
        start in -12_i16..=12,
        lower_root in -12_i16..=8,
        gap in 1_i16..=8,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let lower_root = i64::from(lower_root);
        let upper_root = lower_root + i64::from(gap);
        let mut problem = Problem::default();
        let variable = problem.add_variable("x", Real::from(i64::from(start)));
        problem.variables[variable.0 as usize].lower = Some(Real::from(lower_root + 1));
        problem.add_constraint(Constraint::equality(
            "generated bounded quadratic",
            x.clone() * x.clone()
                - Expr::int(lower_root + upper_root) * x
                + Expr::int(lower_root * upper_root),
        ));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert_eq!(report.iterations, 0);
        prop_assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
        prop_assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
        prop_assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
    }

    #[test]
    fn modified_newton_generated_dragged_weights_report_and_converge(
        start in -12_i16..=12,
        target in -12_i16..=12,
        weight in 1_i16..=8,
    ) {
        let mut problem = Problem::default();
        let x = problem.add_variable("x", Real::from(i64::from(start)));

        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 4,
                damping: Real::zero(),
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                dragged_parameters: vec![DraggedParameterWeight {
                    variable: x,
                    target: Real::from(i64::from(target)),
                    weight: Real::from(i64::from(weight)),
                }],
                ..SolverConfig::default()
            },
        });

        prop_assert_eq!(report.reason, ConvergenceReason::Converged);
        prop_assert!(report.preprocessing.requested);
        prop_assert_eq!(report.preprocessing.dragged_parameter_weights, 1);
        prop_assert_eq!(report.preprocessing.invalid_dragged_parameter_weights, 0);
    }

    #[test]
    fn prepared_univariate_quadratic_generated_coefficients_replay_exactly(
        a in -8_i16..=8,
        b in -8_i16..=8,
        c in -8_i16..=8,
        x_value in -8_i16..=8,
    ) {
        prop_assume!(a != 0);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_value)));
        problem.add_constraint(Constraint::equality(
            "generated univariate quadratic",
            x.clone() * x.clone() * Expr::int(i64::from(a))
                + x * Expr::int(i64::from(b))
                + Expr::int(i64::from(c)),
        ));
        let prepared = PreparedProblem::new(&problem);
        let block = PreparedSolverBlock::new(&prepared);
        let quadratic = prepared.univariate_quadratic_residuals()[0]
            .as_ref()
            .expect("generated univariate quadratic should prepare");
        let expected = i64::from(a) * i64::from(x_value) * i64::from(x_value)
            + i64::from(b) * i64::from(x_value)
            + i64::from(c);

        prop_assert_eq!(prepared.facts().prepared_univariate_quadratic_active_rows, 1);
        prop_assert_eq!(block.facts().prepared_univariate_quadratic_row_count, 1);
        prop_assert_eq!(quadratic.quadratic(), &Real::from(i64::from(a)));
        prop_assert_eq!(quadratic.linear(), &Real::from(i64::from(b)));
        prop_assert_eq!(quadratic.constant(), &Real::from(i64::from(c)));
        prop_assert_eq!(
            quadratic
                .eval_real(problem.variables.as_slice(), context_from_problem(&problem).bindings())
                .unwrap(),
            Real::from(expected)
        );
    }

    #[test]
    fn prepared_multivariate_quadratic_generated_coefficients_replay_exactly(
        ax2 in -4_i16..=4,
        bxy in -4_i16..=4,
        cy2 in -4_i16..=4,
        dx in -8_i16..=8,
        ey in -8_i16..=8,
        f in -16_i16..=16,
        x_value in -8_i16..=8,
        y_value in -8_i16..=8,
    ) {
        prop_assume!(ax2 != 0 || bxy != 0 || cy2 != 0);
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_value)));
        problem.add_variable("y", Real::from(i64::from(y_value)));
        problem.add_constraint(Constraint::equality(
            "generated multivariate quadratic",
            x.clone() * x.clone() * Expr::int(i64::from(ax2))
                + x.clone() * y.clone() * Expr::int(i64::from(bxy))
                + y.clone() * y.clone() * Expr::int(i64::from(cy2))
                + x * Expr::int(i64::from(dx))
                + y * Expr::int(i64::from(ey))
                + Expr::int(i64::from(f)),
        ));
        let prepared = PreparedProblem::new(&problem);
        let block = PreparedSolverBlock::new(&prepared);
        let quadratic = prepared.quadratic_residuals()[0]
            .as_ref()
            .expect("generated multivariate quadratic should prepare");
        let expected = i64::from(ax2) * i64::from(x_value) * i64::from(x_value)
            + i64::from(bxy) * i64::from(x_value) * i64::from(y_value)
            + i64::from(cy2) * i64::from(y_value) * i64::from(y_value)
            + i64::from(dx) * i64::from(x_value)
            + i64::from(ey) * i64::from(y_value)
            + i64::from(f);

        prop_assert_eq!(prepared.facts().prepared_quadratic_active_rows, 1);
        prop_assert_eq!(block.facts().prepared_quadratic_row_count, 1);
        prop_assert_eq!(
            quadratic
                .eval_real(problem.variables.as_slice(), context_from_problem(&problem).bindings())
                .unwrap(),
            Real::from(expected)
        );
        prop_assert_eq!(
            prepared.evaluate_residuals(&context_from_problem(&problem)).unwrap()[0].value.clone(),
            Real::from(expected)
        );
    }

    #[test]
    fn candidate_certification_generated_multivariate_quadratics_replay_exactly(
        ax2 in -4_i16..=4,
        bxy in -4_i16..=4,
        cy2 in -4_i16..=4,
        dx in -8_i16..=8,
        ey in -8_i16..=8,
        x_value in -8_i16..=8,
        y_value in -8_i16..=8,
    ) {
        prop_assume!(ax2 != 0 || bxy != 0 || cy2 != 0);
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let expected = i64::from(ax2) * i64::from(x_value) * i64::from(x_value)
            + i64::from(bxy) * i64::from(x_value) * i64::from(y_value)
            + i64::from(cy2) * i64::from(y_value) * i64::from(y_value)
            + i64::from(dx) * i64::from(x_value)
            + i64::from(ey) * i64::from(y_value);
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_value)));
        problem.add_variable("y", Real::from(i64::from(y_value)));
        problem.add_constraint(Constraint::equality(
            "generated certified quadratic replay",
            x.clone() * x.clone() * Expr::int(i64::from(ax2))
                + x.clone() * y.clone() * Expr::int(i64::from(bxy))
                + y.clone() * y.clone() * Expr::int(i64::from(cy2))
                + x * Expr::int(i64::from(dx))
                + y * Expr::int(i64::from(ey))
                - Expr::int(expected),
        ));
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        let report = certify_candidate(&prepared, &context);

        prop_assert!(prepared.quadratic_residuals()[0].is_some());
        prop_assert_eq!(
            prepared.evaluate_constraint_residual(0, &context).unwrap(),
            Real::zero()
        );
        prop_assert!(report.all_satisfied());
    }

    #[test]
    fn direct_quadratic_solver_generated_integer_roots_replay_exactly(
        first in -16_i16..=16,
        second in -16_i16..=16,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let first = i64::from(first);
        let second = i64::from(second);
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated integer roots",
            x.clone().powi(2)
                - x.clone() * Expr::int(first + second)
                + Expr::int(first * second),
        ));
        let prepared = PreparedProblem::new(&problem);
        let solutions = solve_direct_univariate_quadratic_equalities(&prepared).unwrap();

        prop_assert_eq!(solutions.len(), 1);
        if first == second {
            prop_assert_eq!(&solutions[0].roots, &vec![Real::from(first)]);
        } else {
            prop_assert_eq!(
                &solutions[0].roots,
                &vec![Real::from(first.max(second)), Real::from(first.min(second))]
            );
        }
    }

    #[test]
    fn direct_quadratic_candidate_replay_selects_generated_root_exactly(
        selected in -16_i16..=16,
        rejected in -16_i16..=16,
    ) {
        prop_assume!(selected != rejected);
        let selected = i64::from(selected);
        let rejected = i64::from(rejected);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated integer roots",
            x.clone().powi(2)
                - x.clone() * Expr::int(selected + rejected)
                + Expr::int(selected * rejected),
        ));
        problem.add_constraint(Constraint::equality(
            "select generated root",
            x - Expr::int(selected),
        ));
        let prepared = PreparedProblem::new(&problem);
        let reports =
            certify_direct_univariate_quadratic_roots(&prepared, &context_from_problem(&problem))
                .unwrap();

        prop_assert_eq!(reports.len(), 2);
        prop_assert_eq!(
            reports
                .iter()
                .filter(|report| {
                    report.status
                        == hypersolve::DirectQuadraticCandidateStatus::ReplayCertified
                })
                .count(),
            1
        );
        let certified = reports
            .iter()
            .find(|report| {
                report.status == hypersolve::DirectQuadraticCandidateStatus::ReplayCertified
            })
            .expect("one root should replay exactly");
        prop_assert_eq!(&certified.root, &Some(Real::from(selected)));
        prop_assert!(certified.certification.as_ref().unwrap().all_satisfied());
    }

    #[test]
    fn root_isolation_generated_integer_roots_are_distinct_intervals(
        first in -24_i16..=24,
        second in -24_i16..=24,
    ) {
        prop_assume!(first != second);
        let first = i64::from(first);
        let second = i64::from(second);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated root isolation",
            x.clone().powi(2)
                - x.clone() * Expr::int(first + second)
                + Expr::int(first * second),
        ));

        let reports = isolate_univariate_polynomial_roots(
            &PreparedProblem::new(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(reports.len(), 1);
        prop_assert_eq!(&reports[0].status, &RootIsolationStatus::Isolated);
        prop_assert_eq!(reports[0].intervals.len(), 2);
        for root in [first, second] {
            let root = Real::from(root);
            let containing = reports[0].intervals.iter().any(|interval| {
                if interval.exact_root.as_ref() == Some(&root) {
                    return true;
                }
                let lower_ok = hyperlimit::compare_reals_with_policy(
                    &interval.lower,
                    &root,
                    hyperlimit::PredicatePolicy::default(),
                )
                .value()
                    == Some(std::cmp::Ordering::Less);
                let upper_ok = hyperlimit::compare_reals_with_policy(
                    &root,
                    &interval.upper,
                    hyperlimit::PredicatePolicy::default(),
                )
                .value()
                    == Some(std::cmp::Ordering::Less);
                lower_ok && upper_ok
            });
            prop_assert!(containing);
        }
    }

    #[test]
    fn exact_quadratic_branch_enumeration_generated_roots_count_certified_branches(
        first in -12_i16..=12,
        second in -12_i16..=12,
    ) {
        let first = i64::from(first);
        let second = i64::from(second);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "generated branch roots",
            x.clone().powi(2)
                - x.clone() * Expr::int(first + second)
                + Expr::int(first * second),
        ));
        let prepared = PreparedProblem::new(&problem);
        let report =
            enumerate_direct_univariate_quadratic_branches(&prepared, &context_from_problem(&problem));

        let expected_branch_count = if first == second { 1 } else { 2 };
        prop_assert_eq!(report.certified_branches, expected_branch_count);
        prop_assert_eq!(report.rejected_branches, 0);
        prop_assert_eq!(report.unsupported_rows, 0);
        prop_assert_eq!(report.no_real_root_rows, 0);
        prop_assert_eq!(report.branches.len(), expected_branch_count);
        let all_certified = report.branches.iter().all(|branch| {
            branch.status == hypersolve::ExactBranchStatus::ReplayCertified
                && branch.certification.as_ref().unwrap().all_satisfied()
        });
        prop_assert!(all_certified);
    }

    #[test]
    fn univariate_quadratic_alpha_generated_exact_simple_roots_certify(
        root in -16_i16..=16,
        other_root in -16_i16..=16,
        scale in 1_i16..=8,
    ) {
        prop_assume!(root != other_root);
        let root = i64::from(root);
        let other_root = i64::from(other_root);
        let scale = i64::from(scale);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(root));
        problem.add_constraint(Constraint::equality(
            "generated simple root",
            (x.clone().powi(2)
                - x.clone() * Expr::int(root + other_root)
                + Expr::int(root * other_root))
                * Expr::int(scale),
        ));
        let report = certify_univariate_quadratic_alpha(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.examined_rows, 1);
        prop_assert_eq!(
            &report.rows[0].status,
            &hypersolve::UnivariateQuadraticAlphaStatus::ExactSimpleRoot
        );
        prop_assert!(report.all_examined_rows_certified());
    }

    #[test]
    fn univariate_quadratic_krawczyk_generated_simple_integer_roots_certify(
        root in -16_i16..=16,
        other_root in -16_i16..=16,
    ) {
        prop_assume!(root != other_root);
        let root = i64::from(root);
        let other_root = i64::from(other_root);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(root));
        problem.add_constraint(Constraint::equality(
            "generated quadratic krawczyk root",
            x.clone().powi(2)
                - x.clone() * Expr::int(root + other_root)
                + Expr::int(root * other_root),
        ));
        let report = certify_univariate_quadratic_krawczyk_box(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            &[VariableBall {
                symbol: SymbolId(0),
                radius: Real::zero(),
            }],
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.examined_rows, 1);
        prop_assert_eq!(report.certified_rows, 1);
        prop_assert_eq!(
            &report.rows[0].status,
            &hypersolve::QuadraticKrawczykStatus::CertifiedUniqueRoot
        );
        prop_assert_eq!(report.rows[0].step.clone(), Some(Real::zero()));
        prop_assert!(report.all_examined_rows_certified());
    }

    #[test]
    fn multivariate_quadratic_krawczyk_generated_diagonal_roots_certify(
        x_root in 2_i16..=16,
        y_root in 2_i16..=16,
    ) {
        let x_root = i64::from(x_root);
        let y_root = i64::from(y_root);
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(x_root));
        problem.add_variable("y", Real::from(y_root));
        problem.add_constraint(Constraint::equality(
            "generated x diagonal quadratic",
            x.clone().powi(2) - Expr::int(x_root * x_root),
        ));
        problem.add_constraint(Constraint::equality(
            "generated y diagonal quadratic",
            y.clone().powi(2) - Expr::int(y_root * y_root),
        ));

        let report = certify_multivariate_quadratic_krawczyk_box(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            &[
                VariableBall {
                    symbol: SymbolId(0),
                    radius: Real::from(1),
                },
                VariableBall {
                    symbol: SymbolId(1),
                    radius: Real::from(1),
                },
            ],
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(
            &report.status,
            &MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot
        );
        prop_assert!(report.certified_unique_root());
        prop_assert_eq!(report.variables.len(), 2);
        let all_variables_bounded = report.variables.iter().all(|variable| {
            variable.step == Real::zero()
                && variable.image_radius <= variable.radius
                && variable.contraction_bound < Real::from(1)
        });
        prop_assert!(all_variables_bounded);
    }

    #[test]
    fn univariate_quadratic_alpha_generated_large_positive_offsets_fail_bound(
        center in -4_i16..=4,
        offset in 16_i16..=32,
    ) {
        let center = i64::from(center);
        let offset = i64::from(offset);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(center));
        problem.add_constraint(Constraint::equality(
            "generated far quadratic",
            x.clone().powi(2) - Expr::int(center * center + offset),
        ));
        let report = certify_univariate_quadratic_alpha(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.examined_rows, 1);
        prop_assert!(matches!(
            report.rows[0].status,
            hypersolve::UnivariateQuadraticAlphaStatus::BoundFailed
                | hypersolve::UnivariateQuadraticAlphaStatus::ZeroDerivative
        ));
        prop_assert!(!report.all_examined_rows_certified());
    }

    #[test]
    fn candidate_domain_generated_sqrt_signs_are_certified(
        value in -64_i16..=64,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(value)));
        problem.add_constraint(Constraint::equality("generated sqrt", x.sqrt()));
        let report = certify_candidate_domains(
            &problem,
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.checks.len(), 1);
        prop_assert_eq!(report.checks[0].kind, DomainCheckKind::SqrtOperandNonNegative);
        if value < 0 {
            prop_assert_eq!(
                &report.checks[0].status,
                &DomainCheckStatus::CertifiedInvalid
            );
            prop_assert!(report.has_certified_invalid_domain());
        } else {
            prop_assert_eq!(
                &report.checks[0].status,
                &DomainCheckStatus::CertifiedValid
            );
            prop_assert!(report.all_checks_certified_valid());
        }
    }

    #[test]
    fn simplify_generated_sqrt_and_log_domains_preserve_invalid_nodes(
        sqrt_value in -64_i16..=64,
        log_value in -64_i16..=64,
        unit_value in -2_i16..=2,
    ) {
        let simplified_sqrt = Expr::int(i64::from(sqrt_value)).sqrt().simplify();
        let simplified_log = Expr::int(i64::from(log_value)).log10().simplify();
        let simplified_ln = Expr::int(i64::from(log_value)).ln().simplify();
        let simplified_asin = Expr::int(i64::from(unit_value)).asin().simplify();
        let simplified_acos = Expr::int(i64::from(unit_value)).acos().simplify();
        let simplified_acosh = Expr::int(i64::from(unit_value)).acosh().simplify();
        let simplified_atanh = Expr::int(i64::from(unit_value)).atanh().simplify();

        if sqrt_value < 0 {
            prop_assert!(matches!(simplified_sqrt, Expr::Sqrt(_)));
        } else {
            prop_assert!(matches!(simplified_sqrt, Expr::Constant(_)));
        }
        if log_value <= 0 {
            prop_assert!(matches!(simplified_log, Expr::Log10(_)));
            prop_assert!(matches!(simplified_ln, Expr::Ln(_)));
        } else {
            prop_assert!(matches!(simplified_log, Expr::Constant(_)));
            prop_assert!(matches!(simplified_ln, Expr::Constant(_)));
        }
        if !(-1..=1).contains(&unit_value) {
            prop_assert!(matches!(simplified_asin, Expr::Asin(_)));
            prop_assert!(matches!(simplified_acos, Expr::Acos(_)));
        } else {
            prop_assert!(matches!(simplified_asin, Expr::Constant(_)));
            prop_assert!(matches!(simplified_acos, Expr::Constant(_)));
        }
        if unit_value < 1 {
            prop_assert!(matches!(simplified_acosh, Expr::Acosh(_)));
        } else {
            prop_assert!(matches!(simplified_acosh, Expr::Constant(_)));
        }
        if unit_value <= -1 || unit_value >= 1 {
            prop_assert!(matches!(simplified_atanh, Expr::Atanh(_)));
        } else {
            prop_assert!(matches!(simplified_atanh, Expr::Constant(_)));
        }
    }

    #[test]
    fn candidate_domain_generated_division_and_log_boundaries_are_certified(
        denominator in -64_i16..=64,
        log_operand in -64_i16..=64,
        unit_operand in -3_i16..=3,
    ) {
        let y = Expr::symbol(SymbolId(0), "y");
        let l = Expr::symbol(SymbolId(1), "l");
        let u = Expr::symbol(SymbolId(2), "u");
        let mut problem = Problem::default();
        problem.add_variable("y", Real::from(i64::from(denominator)));
        problem.add_variable("l", Real::from(i64::from(log_operand)));
        problem.add_variable("u", Real::from(i64::from(unit_operand)));
        problem.add_constraint(Constraint::equality("generated reciprocal", Expr::int(1) / y));
        problem.add_constraint(Constraint::equality("generated log", l.clone().log10()));
        problem.add_constraint(Constraint::equality("generated ln", l.ln()));
        problem.add_constraint(Constraint::equality("generated asin", u.clone().asin()));
        problem.add_constraint(Constraint::equality("generated acos", u.clone().acos()));
        problem.add_constraint(Constraint::equality("generated acosh", u.clone().acosh()));
        problem.add_constraint(Constraint::equality("generated atanh", u.atanh()));
        let report = certify_candidate_domains(
            &problem,
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.checks.len(), 7);
        let division_status = &report.checks[0].status;
        let log_status = &report.checks[1].status;
        let ln_status = &report.checks[2].status;
        let asin_status = &report.checks[3].status;
        let acos_status = &report.checks[4].status;
        let acosh_status = &report.checks[5].status;
        let atanh_status = &report.checks[6].status;
        prop_assert_eq!(
            *division_status == DomainCheckStatus::CertifiedValid,
            denominator != 0
        );
        prop_assert_eq!(
            *division_status == DomainCheckStatus::CertifiedInvalid,
            denominator == 0
        );
        prop_assert_eq!(
            *log_status == DomainCheckStatus::CertifiedValid,
            log_operand > 0
        );
        prop_assert_eq!(
            *log_status == DomainCheckStatus::CertifiedInvalid,
            log_operand <= 0
        );
        prop_assert_eq!(*ln_status == DomainCheckStatus::CertifiedValid, log_operand > 0);
        prop_assert_eq!(*ln_status == DomainCheckStatus::CertifiedInvalid, log_operand <= 0);
        prop_assert_eq!(
            *asin_status == DomainCheckStatus::CertifiedValid,
            (-1..=1).contains(&unit_operand)
        );
        prop_assert_eq!(
            *asin_status == DomainCheckStatus::CertifiedInvalid,
            !(-1..=1).contains(&unit_operand)
        );
        prop_assert_eq!(
            *acos_status == DomainCheckStatus::CertifiedValid,
            (-1..=1).contains(&unit_operand)
        );
        prop_assert_eq!(
            *acos_status == DomainCheckStatus::CertifiedInvalid,
            !(-1..=1).contains(&unit_operand)
        );
        prop_assert_eq!(
            *acosh_status == DomainCheckStatus::CertifiedValid,
            unit_operand >= 1
        );
        prop_assert_eq!(
            *acosh_status == DomainCheckStatus::CertifiedInvalid,
            unit_operand < 1
        );
        prop_assert_eq!(
            *atanh_status == DomainCheckStatus::CertifiedValid,
            unit_operand > -1 && unit_operand < 1
        );
        prop_assert_eq!(
            *atanh_status == DomainCheckStatus::CertifiedInvalid,
            unit_operand <= -1 || unit_operand >= 1
        );
    }

    #[test]
    fn quadratic_interval_generated_positive_balls_certify_violation(
        a in 1_i16..=8,
        x_value in 10_i16..=20,
        radius in 0_i16..=2,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_value)));
        problem.add_constraint(Constraint::equality(
            "generated positive quadratic ball",
            x.clone() * x * Expr::int(i64::from(a)),
        ));
        let prepared = PreparedProblem::new(&problem);
        let report = certify_quadratic_interval_candidate(
            &prepared,
            &context_from_problem(&problem),
            &[VariableBall {
                symbol: SymbolId(0),
                radius: Real::from(i64::from(radius)),
            }],
            hyperlimit::PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.certified_violation_rows, 1);
        let ball_certified_positive = matches!(
            report.rows[0].status,
            CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Positive
            }
        );
        prop_assert_eq!(ball_certified_positive, true);
    }

    #[test]
    fn interval_box_reports_generated_affine_status(
        center in -64_i16..=64,
        target in -64_i16..=64,
        radius in 0_i16..=8,
    ) {
        let center = i64::from(center);
        let target = i64::from(target);
        let radius = i64::from(radius);
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(center));
        problem.add_constraint(Constraint::equality("generated affine box", x - Expr::int(target)));
        let report = certify_interval_box_candidate(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            &[VariableBall {
                symbol: SymbolId(0),
                radius: Real::from(radius),
            }],
            IntervalBoxCertificationPackage::Affine,
            hyperlimit::PredicatePolicy::default(),
        );
        let delta = (center - target).abs();

        prop_assert_eq!(
            report.status,
            if delta == 0 && radius == 0 {
                IntervalBoxCertificationStatus::Certified
            } else if delta > radius {
                IntervalBoxCertificationStatus::Violation
            } else {
                IntervalBoxCertificationStatus::Unknown
            }
        );
        prop_assert_eq!(report.variable_balls.len(), 1);
        prop_assert!(report.certification.is_some());
    }

    #[test]
    fn affine_krawczyk_generated_diagonal_systems_certify_exact_roots(
        x_root in -32_i16..=32,
        y_root in -32_i16..=32,
        x_step in -8_i16..=8,
        y_step in -8_i16..=8,
        x_extra_radius in 0_i16..=4,
        y_extra_radius in 0_i16..=4,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_root + x_step)));
        problem.add_variable("y", Real::from(i64::from(y_root + y_step)));
        problem.add_constraint(Constraint::equality(
            "x diagonal root",
            x - Expr::int(i64::from(x_root)),
        ));
        problem.add_constraint(Constraint::equality(
            "y diagonal root",
            y - Expr::int(i64::from(y_root)),
        ));

        let x_radius = i64::from(x_step).abs() + i64::from(x_extra_radius);
        let y_radius = i64::from(y_step).abs() + i64::from(y_extra_radius);
        let report = certify_affine_krawczyk_box(
            &PreparedProblem::new(&problem),
            &context_from_problem(&problem),
            &[
                VariableBall {
                    symbol: SymbolId(0),
                    radius: Real::from(x_radius),
                },
                VariableBall {
                    symbol: SymbolId(1),
                    radius: Real::from(y_radius),
                },
            ],
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(
            report.status,
            hypersolve::AffineKrawczykStatus::CertifiedUniqueRoot
        );
        prop_assert_eq!(report.steps[0].certified_root.clone(), Real::from(i64::from(x_root)));
        prop_assert_eq!(report.steps[1].certified_root.clone(), Real::from(i64::from(y_root)));
        prop_assert_eq!(report.steps[0].step.clone(), Real::from(-i64::from(x_step)));
        prop_assert_eq!(report.steps[1].step.clone(), Real::from(-i64::from(y_step)));
    }

    #[test]
    fn direct_affine_system_generated_diagonal_rows_replay_exactly(
        x_root in -64_i16..=64,
        y_root in -64_i16..=64,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::zero());
        problem.add_variable("y", Real::zero());
        problem.add_constraint(Constraint::equality(
            "generated direct x root",
            x - Expr::int(i64::from(x_root)),
        ));
        problem.add_constraint(Constraint::equality(
            "generated direct y root",
            y - Expr::int(i64::from(y_root)),
        ));

        let prepared = PreparedProblem::new(&problem);
        let report = solve_direct_affine_system(&prepared);

        prop_assert!(report.solved());
        prop_assert_eq!(report.assignments.len(), 2);
        prop_assert_eq!(report.assignments[0].value.clone(), Real::from(i64::from(x_root)));
        prop_assert_eq!(report.assignments[1].value.clone(), Real::from(i64::from(y_root)));
        let mut context = context_from_problem(&problem);
        for assignment in &report.assignments {
            context.bind(assignment.symbol, assignment.value.clone());
        }
        let certification = certify_candidate(&prepared, &context);
        prop_assert!(certification.all_satisfied());
    }

    #[test]
    fn multivariate_quadratic_interval_generated_positive_balls_certify_violation(
        x_value in 10_i16..=20,
        y_value in 10_i16..=20,
        x_radius in 0_i16..=2,
        y_radius in 0_i16..=2,
    ) {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(i64::from(x_value)));
        problem.add_variable("y", Real::from(i64::from(y_value)));
        problem.add_constraint(Constraint::equality(
            "generated positive cross term ball",
            x * y,
        ));
        let prepared = PreparedProblem::new(&problem);
        let report = certify_multivariate_quadratic_interval_candidate(
            &prepared,
            &context_from_problem(&problem),
            &[
                VariableBall {
                    symbol: SymbolId(0),
                    radius: Real::from(i64::from(x_radius)),
                },
                VariableBall {
                    symbol: SymbolId(1),
                    radius: Real::from(i64::from(y_radius)),
                },
            ],
            hyperlimit::PredicatePolicy::default(),
        ).unwrap();

        let ball_certified_positive = matches!(
            report.rows[0].status,
            CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Positive
            }
        );
        prop_assert_eq!(report.certified_violation_rows, 1);
        prop_assert_eq!(ball_certified_positive, true);
    }

    #[test]
    fn substitution_cycle_validation_sums_generated_offsets_exactly(
        first in -64_i16..=64,
        second in -64_i16..=64,
    ) {
        let substitutions = vec![
            EqualitySubstitution {
                constraint_index: 0,
                left: SymbolId(0),
                right: SymbolId(1),
                offset: Real::from(i64::from(first)),
            },
            EqualitySubstitution {
                constraint_index: 1,
                left: SymbolId(1),
                right: SymbolId(0),
                offset: Real::from(i64::from(second)),
            },
        ];

        let report = validate_equality_substitutions(&substitutions);
        let expected_offset = Real::from(i64::from(first) + i64::from(second));
        let found_expected_cycle = report.problems.iter().any(|problem| matches!(
            problem,
            EqualitySubstitutionProblem::DirectedCycle {
                symbols,
                net_offset,
                consistent,
            } if symbols == &vec![SymbolId(0), SymbolId(1)]
                && net_offset == &expected_offset
                && *consistent == (expected_offset == Real::zero())
        ));

        prop_assert!(found_expected_cycle);
        prop_assert_eq!(
            report.has_inconsistency(),
            expected_offset != Real::zero()
        );
    }

    #[test]
    fn substitution_class_elimination_carries_generated_offsets(
        x_to_y_offset in -20_i16..=20,
        z_to_x_offset in -20_i16..=20,
        row_z_coeff in -8_i16..=8,
        row_y_coeff in -8_i16..=8,
        row_constant in -32_i16..=32,
    ) {
        prop_assume!(!(row_z_coeff == 1 && row_y_coeff == -1));
        prop_assume!(!(row_z_coeff == -1 && row_y_coeff == 1));
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let z = Expr::symbol(SymbolId(2), "z");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(0));
        problem.add_variable("y", Real::from(0));
        problem.add_variable("z", Real::from(0));
        problem.add_constraint(Constraint::equality(
            "x to y",
            x.clone() - y.clone() - Expr::int(i64::from(x_to_y_offset)),
        ));
        problem.add_constraint(Constraint::equality(
            "z to x",
            z.clone() - x.clone() - Expr::int(i64::from(z_to_x_offset)),
        ));
        problem.add_constraint(Constraint::equality(
            "generated reduced row",
            z * Expr::int(i64::from(row_z_coeff))
                + y * Expr::int(i64::from(row_y_coeff))
                + Expr::int(i64::from(row_constant)),
        ));

        let prepared = PreparedProblem::new(&problem);
        let substitutions = hypersolve::find_equality_substitutions(&prepared).unwrap();
        let classes = hypersolve::build_equality_substitution_classes(&substitutions).unwrap();
        let report = eliminate_affine_rows_with_substitution_classes(&prepared, &classes);
        let reduced = &report.rows[2];

        let expected_coefficient = i64::from(row_z_coeff) + i64::from(row_y_coeff);
        let expected_constant = i64::from(row_constant)
            + i64::from(row_z_coeff) * i64::from(z_to_x_offset)
            - i64::from(row_y_coeff) * i64::from(x_to_y_offset);
        let expected_variable_rows = usize::from(expected_coefficient != 0);
        let expected_zero_rows = 2 + usize::from(expected_coefficient == 0 && expected_constant == 0);
        let expected_contradiction_rows =
            usize::from(expected_coefficient == 0 && expected_constant != 0);
        prop_assert_eq!(report.affine_rows_considered, 3);
        prop_assert_eq!(report.reduced_variable_rows, expected_variable_rows);
        prop_assert_eq!(report.reduced_zero_rows, expected_zero_rows);
        prop_assert_eq!(report.reduced_contradiction_rows, expected_contradiction_rows);
        prop_assert_eq!(report.reduced_unknown_constant_rows, 0);
        prop_assert_eq!(reduced.constant.clone(), Real::from(expected_constant));
        if expected_coefficient == 0 {
            prop_assert!(reduced.coefficients.is_empty());
        } else {
            prop_assert_eq!(
                reduced.coefficients.clone(),
                vec![(SymbolId(0), Real::from(expected_coefficient))]
            );
        }
    }

    #[test]
    fn substitution_class_application_reconstructs_generated_contexts_exactly(
        y_offset in -64_i16..=64,
        z_offset in -64_i16..=64,
        representative_value in -128_i16..=128,
    ) {
        let y_offset = i64::from(y_offset);
        let z_offset = i64::from(z_offset);
        let representative_value = i64::from(representative_value);
        let substitutions = vec![
            EqualitySubstitution {
                constraint_index: 0,
                left: SymbolId(1),
                right: SymbolId(0),
                offset: Real::from(y_offset),
            },
            EqualitySubstitution {
                constraint_index: 1,
                left: SymbolId(2),
                right: SymbolId(1),
                offset: Real::from(z_offset),
            },
        ];
        let classes = hypersolve::build_equality_substitution_classes(&substitutions).unwrap();
        let mut context = hypersolve::EvaluationContext::default();
        context.bind(
            SymbolId(2),
            Real::from(representative_value + y_offset + z_offset),
        );

        let report = apply_equality_substitution_classes(&mut context, &classes);

        prop_assert!(report.all_consistent());
        prop_assert_eq!(report.applied_bindings, 3);
        prop_assert_eq!(context.bindings().get(&SymbolId(0)), Some(&Real::from(representative_value)));
        prop_assert_eq!(
            context.bindings().get(&SymbolId(1)),
            Some(&Real::from(representative_value + y_offset))
        );
        prop_assert_eq!(
            context.bindings().get(&SymbolId(2)),
            Some(&Real::from(representative_value + y_offset + z_offset))
        );

        let mut conflict = hypersolve::EvaluationContext::default();
        conflict.bind(SymbolId(0), Real::from(representative_value));
        conflict.bind(SymbolId(1), Real::from(representative_value + y_offset + 1));
        let conflict_report = apply_equality_substitution_classes(&mut conflict, &classes);
        prop_assert!(!conflict_report.all_consistent());
        prop_assert_eq!(conflict_report.inconsistent_classes, 1);
        prop_assert_eq!(conflict_report.applied_bindings, 0);
    }

    #[test]
    fn geometry_domain_generated_squared_distance_certifies_exactly(
        ax in -32_i16..=32,
        ay in -32_i16..=32,
        bx in -32_i16..=32,
        by in -32_i16..=32,
    ) {
        let mut problem = Problem::default();
        let ax_id = problem.add_variable("ax", Real::from(i64::from(ax)));
        let ay_id = problem.add_variable("ay", Real::from(i64::from(ay)));
        let bx_id = problem.add_variable("bx", Real::from(i64::from(bx)));
        let by_id = problem.add_variable("by", Real::from(i64::from(by)));
        let dx = i64::from(ax) - i64::from(bx);
        let dy = i64::from(ay) - i64::from(by);
        let target_squared = dx * dx + dy * dy;
        problem.add_constraint(squared_distance_equation(
            "generated squared distance",
            SolverPoint2::new(ax_id, ay_id),
            SolverPoint2::new(bx_id, by_id),
            Real::from(target_squared),
        ));

        prop_assert!(
            certify_candidate(&PreparedProblem::new(&problem), &context_from_problem(&problem))
                .all_satisfied()
        );
    }

    #[test]
    fn bareiss_generated_triangular_determinants_are_exact(
        a in 1_i16..=32,
        b in 1_i16..=32,
        c in -32_i16..=32,
    ) {
        let a = i64::from(a);
        let b = i64::from(b);
        let c = i64::from(c);
        let report = determinant_bareiss(
            &[
                vec![Real::from(a), Real::from(c)],
                vec![Real::zero(), Real::from(b)],
            ],
            -64,
        ).unwrap();

        prop_assert_eq!(report.determinant, Real::from(a * b));
    }

    #[test]
    fn bareiss_generated_diagonal_systems_replay_exactly(
        a in 1_i16..=32,
        b in 1_i16..=32,
        x in -64_i16..=64,
        y in -64_i16..=64,
    ) {
        let a = i64::from(a);
        let b = i64::from(b);
        let x = i64::from(x);
        let y = i64::from(y);
        let report = solve_dense_linear_system_bareiss(
            &[
                vec![Real::from(a), Real::zero()],
                vec![Real::zero(), Real::from(b)],
            ],
            &[Real::from(a * x), Real::from(b * y)],
            -64,
        ).unwrap();

        prop_assert_eq!(report.solution, vec![Real::from(x), Real::from(y)]);
        prop_assert!(report.residual_replay.accepted);
    }

    #[test]
    fn resultant_generated_linear_roots_match_exact_difference(
        left_root in -64_i16..=64,
        right_root in -64_i16..=64,
    ) {
        let left_root = i64::from(left_root);
        let right_root = i64::from(right_root);
        let report = resultant_univariate_polynomials(
            &[Real::from(-left_root), Real::one()],
            &[Real::from(-right_root), Real::one()],
            -64,
        ).unwrap();

        prop_assert_eq!(report.resultant, Real::from(left_root - right_root));
    }

    #[test]
    fn resultant_generated_common_factor_is_zero(
        shared_root in -32_i16..=32,
        other_root in -32_i16..=32,
    ) {
        let shared_root = i64::from(shared_root);
        let other_root = i64::from(other_root);
        let report = resultant_univariate_polynomials(
            &[Real::from(-shared_root), Real::one()],
            &[
                Real::from(shared_root * other_root),
                Real::from(-(shared_root + other_root)),
                Real::one(),
            ],
            -64,
        ).unwrap();

        prop_assert_eq!(report.resultant, Real::zero());
    }

    #[test]
    fn subresultant_generated_common_factor_chain_is_nonconstant(
        shared_root in -32_i16..=32,
        other_root in -32_i16..=32,
    ) {
        let shared_root = i64::from(shared_root);
        let other_root = i64::from(other_root);
        let report = subresultant_chain_univariate_polynomials(
            &[
                Real::from(shared_root * other_root),
                Real::from(-(shared_root + other_root)),
                Real::one(),
            ],
            &[Real::from(-shared_root), Real::one()],
            -64,
        ).unwrap();

        prop_assert!(report.has_nonconstant_common_factor);
        prop_assert_eq!(report.last_nonzero_degree, 1);
    }

}
