use hyperreal::Real;
use hypersolve::eval::EvalError;
use hypersolve::jacobian::{FiniteDifferenceConfig, finite_difference_jacobian};
use hypersolve::{Constraint, EvaluationContext, Expr, ExprEvalError, Problem, SymbolId};

#[test]
fn weighted_polynomial_columns_follow_the_active_bindings() {
    for x in -3..=3 {
        for y in -3..=3 {
            for fixed_y in [false, true] {
                let mut problem = Problem::default();
                problem.add_variable("x", Real::from(x));
                problem.add_variable("y", Real::from(if fixed_y { y + 5 } else { y }));
                problem.variables[1].fixed = fixed_y;
                problem.add_variable("unused", Real::one());

                let sx = Expr::symbol(SymbolId(0), "x");
                let sy = Expr::symbol(SymbolId(1), "y");
                let polynomial = sx.clone().powi(2) + Expr::int(3) * sx.clone() * sy.clone()
                    - sy.clone().powi(2);
                let mut row = Constraint::equality("weighted polynomial", polynomial);
                row.weight = Real::from(2);
                problem.add_constraint(row);
                let mut inactive =
                    Constraint::equality("inactive", Expr::symbol(SymbolId(2), "unused"));
                inactive.active = false;
                problem.add_constraint(inactive);
                problem.add_constraint(Constraint::equality("linear", sx - Expr::int(2) * sy));

                let mut context = EvaluationContext::default();
                context.bind(SymbolId(0), Real::from(x));
                context.bind(SymbolId(1), Real::from(y));
                let original = context.clone();
                for h in [-2, -1, 1, 2] {
                    let result = finite_difference_jacobian(
                        &problem,
                        &context,
                        FiniteDifferenceConfig {
                            step: Real::from(h),
                        },
                    )
                    .unwrap();
                    // Exact forward differences include +h for x² and -h for -y².
                    let want = [
                        [
                            2 * (2 * x + h + 3 * y),
                            if fixed_y { 0 } else { 2 * (3 * x - 2 * y - h) },
                            0,
                        ],
                        [1, if fixed_y { 0 } else { -2 }, 0],
                    ];
                    assert_eq!(result.len(), want.len());
                    for (actual, expected) in result.iter().zip(want) {
                        assert_eq!(actual.len(), expected.len());
                        for (column, (value, n)) in actual.iter().zip(expected).enumerate() {
                            // Fixed columns keep their initialized +0. An active zero
                            // difference divided by a negative step produces -0.
                            let expected_value = if fixed_y && column == 1 {
                                0.0
                            } else {
                                f64::from(n * h) / f64::from(h)
                            };
                            assert_eq!(
                                value.to_bits(),
                                expected_value.to_bits(),
                                "x={x}, y={y}, h={h}, fixed_y={fixed_y}"
                            );
                        }
                    }
                    assert_eq!(context.bindings(), original.bindings());
                    assert_eq!(context.generation(), original.generation());
                }
            }
        }
    }
}

#[test]
fn a_missing_active_coordinate_preserves_the_error_and_context() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::one());
    problem.add_constraint(Constraint::equality(
        "active square",
        Expr::symbol(SymbolId(0), "x").powi(2),
    ));
    let context = EvaluationContext::default();
    let result = finite_difference_jacobian(
        &problem,
        &context,
        FiniteDifferenceConfig { step: Real::one() },
    );
    assert!(
        matches!(result, Err(EvalError::Expr(ExprEvalError::UnboundSymbol(symbol))) if symbol.id == SymbolId(0))
    );
    assert!(context.bindings().is_empty());
    assert_eq!(context.generation(), 0);
}
