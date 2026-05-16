use hyperreal::RealSign;
use hypersolve::{Constraint, Expr, PreparedProblem, Problem, SymbolId, context_from_problem};
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
}
