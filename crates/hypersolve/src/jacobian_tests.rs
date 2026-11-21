use super::{FiniteDifferenceConfig, finite_difference_jacobian};
use crate::{Constraint, EvaluationContext, Expr, Problem, SymbolId, context_from_problem};
use hyperreal::Real;

#[test]
fn finite_differences_follow_rebound_context_coordinates() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::one());
    problem.add_constraint(Constraint::equality(
        "square",
        Expr::symbol(SymbolId(0), "x").powi(2),
    ));
    let mut context = context_from_problem(&problem);
    for center in [1, 3, -3, 0, -1] {
        context.bind(SymbolId(0), Real::from(center));
        for step in [1, -1] {
            let run = || {
                finite_difference_jacobian(
                    &problem,
                    &context,
                    FiniteDifferenceConfig {
                        step: Real::from(step),
                    },
                )
                .unwrap()
            };
            #[cfg(feature = "dispatch-trace")]
            let matrix = {
                hyperreal::dispatch_trace::reset();
                let matrix = hyperreal::dispatch_trace::with_recording(run);
                let trace = hyperreal::dispatch_trace::take_trace();
                // This numerical proposal must not introduce proof decisions.
                assert_eq!(trace.layer_count("hyperlimit"), 0);
                assert_eq!(trace.operation_count("real", "add"), 1);
                assert_eq!(trace.operation_count("real", "mul"), 2);
                assert_eq!(trace.operation_count("real", "powi"), 2);
                assert_eq!(trace.operation_count("real", "structural_facts"), 2);
                eprintln!("JACOBIAN_TRACE {center} {step} {trace:?}");
                matrix
            };
            #[cfg(not(feature = "dispatch-trace"))]
            let matrix = run();
            // ((c+h)^2-c^2)/h = 2*c+h. Every value is exactly binary representable.
            let expected = f64::from(2 * center + step);
            assert_eq!(matrix.len(), 1);
            assert_eq!(matrix[0].len(), 1);
            eprintln!("center={center} step={step}: observed={:?}", matrix[0][0]);
            assert_eq!(matrix[0][0].to_bits(), expected.to_bits());
        }
    }
    assert_eq!(problem.variables[0].value, Real::one());
}

#[test]
fn context_perturbation_preserves_fixed_unused_and_inactive_roles() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::one());
    problem.add_variable("fixed", Real::from(2));
    problem.variables[1].fixed = true;
    problem.add_variable("unused", Real::from(4));
    let x = Expr::symbol(SymbolId(0), "x");
    let mut row = Constraint::equality(
        "weighted square",
        x.clone().powi(2) + Expr::symbol(SymbolId(1), "fixed"),
    );
    row.weight = Real::from(2);
    problem.add_constraint(row);
    let mut inactive = Constraint::equality("inactive", x);
    inactive.active = false;
    problem.add_constraint(inactive);
    let mut context = EvaluationContext::default();
    context.bind(SymbolId(0), Real::from(3));
    context.bind(SymbolId(1), Real::from(10));
    let before = context.clone();
    let matrix = finite_difference_jacobian(
        &problem,
        &context,
        FiniteDifferenceConfig { step: Real::one() },
    )
    .unwrap();
    // 2 * ((4^2+10)-(3^2+10)) = 14; fixed and unused columns remain zero.
    assert_eq!(matrix.len(), 1);
    assert_eq!(matrix[0].len(), 3);
    assert_eq!(matrix[0][0].to_bits(), 14.0_f64.to_bits());
    assert_eq!(matrix[0][1].to_bits(), 0.0_f64.to_bits());
    assert_eq!(matrix[0][2].to_bits(), 0.0_f64.to_bits());
    assert_eq!(context.bindings(), before.bindings());
    assert_eq!(context.generation(), before.generation());
    assert!(!context.bindings().contains_key(&SymbolId(2)));
}

#[test]
fn missing_used_binding_still_fails_before_perturbation() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::one());
    problem.add_constraint(Constraint::equality(
        "used but unbound",
        Expr::symbol(SymbolId(0), "x"),
    ));
    let result = finite_difference_jacobian(
        &problem,
        &EvaluationContext::default(),
        FiniteDifferenceConfig { step: Real::one() },
    );
    assert!(matches!(
        result,
        Err(crate::eval::EvalError::Expr(
            crate::symbolic::ExprEvalError::UnboundSymbol(_)
        ))
    ));
}
