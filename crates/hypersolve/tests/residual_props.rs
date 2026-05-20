use hyperreal::{Real, RealSign};
use hypersolve::{
    CertifiedCandidateStatus, Constraint, DomainCheckKind, DomainCheckStatus, EqualitySubstitution,
    EqualitySubstitutionProblem, Expr, PreparedProblem, PreparedSolverBlock, Problem,
    ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport, RectangularRegion,
    SolverBlockRowKind, SolverPoint2, SymbolId, VariableBall, center_clearance_squared_constraint,
    certify_affine_krawczyk_box, certify_candidate, certify_candidate_domains,
    certify_multivariate_quadratic_interval_candidate, certify_quadratic_interval_candidate,
    certify_univariate_quadratic_alpha, context_from_problem, differential_pair_skew_equation,
    eliminate_affine_rows_with_substitution_classes, rectangular_difference_area_equation,
    report_lossy_adapter_only_candidate, solve_direct_univariate_quadratic_equalities,
    squared_distance_equation, validate_equality_substitutions,
};
use proptest::prelude::*;

fn expected_sign(value: i64) -> RealSign {
    match value.cmp(&0) {
        std::cmp::Ordering::Less => RealSign::Negative,
        std::cmp::Ordering::Equal => RealSign::Zero,
        std::cmp::Ordering::Greater => RealSign::Positive,
    }
}

fn point(x: i64, y: i64) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(Real::from(x), Real::from(y))
}

fn rect(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> RectangularRegion {
    RectangularRegion::new(point(min_x, min_y), point(max_x, max_y))
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
    fn candidate_domain_generated_division_and_log_boundaries_are_certified(
        denominator in -64_i16..=64,
        log_operand in -64_i16..=64,
    ) {
        let y = Expr::symbol(SymbolId(0), "y");
        let l = Expr::symbol(SymbolId(1), "l");
        let mut problem = Problem::default();
        problem.add_variable("y", Real::from(i64::from(denominator)));
        problem.add_variable("l", Real::from(i64::from(log_operand)));
        problem.add_constraint(Constraint::equality("generated reciprocal", Expr::int(1) / y));
        problem.add_constraint(Constraint::equality("generated log", l.log10()));
        let report = certify_candidate_domains(
            &problem,
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy::default(),
        );

        prop_assert_eq!(report.checks.len(), 2);
        let division_status = &report.checks[0].status;
        let log_status = &report.checks[1].status;
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
    fn pcb_domain_generated_clearance_threshold_matches_squared_distance(
        ax in -16_i16..=16,
        ay in -16_i16..=16,
        bx in -16_i16..=16,
        by in -16_i16..=16,
        required in 0_i16..=8,
    ) {
        let mut problem = Problem::default();
        let ax_id = problem.add_variable("ax", Real::from(i64::from(ax)));
        let ay_id = problem.add_variable("ay", Real::from(i64::from(ay)));
        let bx_id = problem.add_variable("bx", Real::from(i64::from(bx)));
        let by_id = problem.add_variable("by", Real::from(i64::from(by)));
        problem.add_constraint(center_clearance_squared_constraint(
            "generated center clearance",
            SolverPoint2::new(ax_id, ay_id),
            SolverPoint2::new(bx_id, by_id),
            Real::from(i64::from(required)),
        ));
        let dx = i64::from(ax) - i64::from(bx);
        let dy = i64::from(ay) - i64::from(by);
        let expected_satisfied = dx * dx + dy * dy >= i64::from(required) * i64::from(required);
        let report = certify_candidate(&PreparedProblem::new(&problem), &context_from_problem(&problem));

        prop_assert_eq!(report.all_satisfied(), expected_satisfied);
        prop_assert_eq!(report.has_certified_violation(), !expected_satisfied);
    }

    #[test]
    fn pcb_domain_generated_differential_pair_skew_matches_lengths(
        first in 0_i16..=512,
        second in 0_i16..=512,
        target in -64_i16..=64,
    ) {
        let mut problem = Problem::default();
        let first_id = problem.add_variable("first_length", Real::from(i64::from(first)));
        let second_id = problem.add_variable("second_length", Real::from(i64::from(second)));
        problem.add_constraint(differential_pair_skew_equation(
            "generated differential pair skew",
            Expr::symbol(SymbolId(first_id.0), "first_length"),
            Expr::symbol(SymbolId(second_id.0), "second_length"),
            Real::from(i64::from(target)),
        ));
        let report = certify_candidate(&PreparedProblem::new(&problem), &context_from_problem(&problem));
        let expected_satisfied = i64::from(first) - i64::from(second) == i64::from(target);

        prop_assert_eq!(report.all_satisfied(), expected_satisfied);
        prop_assert_eq!(report.has_certified_violation(), !expected_satisfied);
    }

    #[test]
    fn toolpath_domain_generated_rectangular_difference_area_certifies_exactly(
        width in 1_i16..=32,
        height in 1_i16..=32,
        cut_width in 1_i16..=32,
        cut_height in 1_i16..=32,
    ) {
        let width = i64::from(width);
        let height = i64::from(height);
        let cut_width = cut_width.min(width as i16) as i64;
        let cut_height = cut_height.min(height as i16) as i64;
        let subject = rect(0, 0, width, height);
        let removed = rect(0, 0, cut_width, cut_height);
        let remainder = vec![
            rect(cut_width, 0, width, height),
            rect(0, cut_height, cut_width, height),
        ];
        let mut problem = Problem::default();
        problem.add_constraint(rectangular_difference_area_equation(
            "generated rectangular difference area",
            subject,
            Some(removed),
            remainder,
        ));

        prop_assert!(
            certify_candidate(&PreparedProblem::new(&problem), &context_from_problem(&problem))
                .all_satisfied()
        );
    }
}
