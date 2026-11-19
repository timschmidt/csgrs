//! Dense proposal Jacobians differentiate the weighted, one-sided residual.
use hyperreal::Real;
use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_with_analysis};
use hypersolve::{
    Constraint, ConstraintKind, ConvergenceReason, Expr, Problem, SolverConfig, SolverState,
    SymbolId, context_from_problem, solve_damped_least_squares,
};

#[test]
fn signed_rows_follow_the_weighted_hinge_and_active_constraint_order() {
    for x in -2..=2 {
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(x));
        problem.add_variable("y", Real::zero());
        problem.add_variable("unused", Real::one());
        let residual = Expr::int(3) * Expr::symbol(SymbolId(0), "x")
            - Expr::int(5) * Expr::symbol(SymbolId(1), "y");
        let mut omitted = Constraint::equality("omitted", residual.clone());
        omitted.active = false;
        problem.add_constraint(omitted);

        let mut expected = Vec::new();
        for kind in [
            ConstraintKind::Equality,
            ConstraintKind::GreaterOrEqual,
            ConstraintKind::LessOrEqual,
            ConstraintKind::Soft,
        ] {
            for weight in [-2, 0, 3] {
                let mut row = Constraint::equality("weighted", residual.clone());
                row.kind = kind;
                row.weight = Real::from(weight);
                problem.add_constraint(row);
                // The documented inactive branch at zero contributes a zero row.
                let slope = match kind {
                    ConstraintKind::GreaterOrEqual if x < 0 => -weight,
                    ConstraintKind::LessOrEqual if x > 0 => weight,
                    ConstraintKind::Equality | ConstraintKind::Soft => weight,
                    _ => 0,
                };
                expected.push(vec![f64::from(3 * slope), f64::from(-5 * slope), 0.0]);
            }
        }
        let context = context_from_problem(&problem);
        assert_eq!(symbolic_jacobian(&problem, &context).unwrap(), expected);
        assert_eq!(
            symbolic_jacobian_with_analysis(&problem.analyze(), &context).unwrap(),
            expected
        );
    }
}

#[test]
fn equivalent_inequalities_converge_at_the_same_boundary() {
    for kind in [ConstraintKind::LessOrEqual, ConstraintKind::GreaterOrEqual] {
        let mut problem = Problem::default();
        problem.add_variable("x", Real::from(-3));
        let x = Expr::symbol(SymbolId(0), "x");
        let residual = if kind == ConstraintKind::GreaterOrEqual {
            x
        } else {
            -x
        };
        let mut row = Constraint::equality("x >= 0", residual);
        row.kind = kind;
        row.weight = Real::from(2);
        problem.add_constraint(row);
        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                damping: Real::zero(),
                ..SolverConfig::default()
            },
        });
        assert_eq!(report.reason, ConvergenceReason::Converged);
        assert_eq!(report.iterations, 1);
        assert_eq!(report.residuals.len(), 1);
        assert_eq!(report.residuals[0].value, Real::zero());
    }
}

#[test]
fn orientation_survives_the_dense_projection_boundary() {
    for (exponent, bits) in [(-1074, 1_u64), (-1076, 0)] {
        let scale = Real::from(2).powi(exponent.into()).unwrap();
        for negative in [false, true] {
            let mut problem = Problem::default();
            problem.add_variable("x", Real::from(-1));
            problem.add_variable("unused", Real::zero());
            let mut row = Constraint::equality("x >= 0", Expr::symbol(SymbolId(0), "x"));
            row.kind = ConstraintKind::GreaterOrEqual;
            row.weight = if negative {
                -scale.clone()
            } else {
                scale.clone()
            };
            problem.add_constraint(row);
            let context = context_from_problem(&problem);
            let expected = if negative { bits } else { bits | (1 << 63) };
            for matrix in [
                symbolic_jacobian(&problem, &context).unwrap(),
                symbolic_jacobian_with_analysis(&problem.analyze(), &context).unwrap(),
            ] {
                assert_eq!(matrix[0][0].to_bits(), expected);
                assert_eq!(matrix[0][1].to_bits(), 0);
            }
        }
    }
}
