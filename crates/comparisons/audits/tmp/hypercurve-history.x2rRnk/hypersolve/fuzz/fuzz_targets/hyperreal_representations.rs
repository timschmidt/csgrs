//! Solver expression evaluation over every pair of Hyperreal representations.

#![no_main]

use std::collections::HashMap;

use hyperreal::{Rational, Real, StructuralKind};
use hypersolve::{Constraint, EvaluationContext, Expr, Problem, SymbolId, evaluate_residuals};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for left in &values {
        for right in &values {
            let x = SymbolId(0);
            let y = SymbolId(1);
            let mut bindings = HashMap::new();
            bindings.insert(x, left.clone());
            bindings.insert(y, right.clone());

            let expressions = [
                Expr::symbol(x, "x") + Expr::symbol(y, "y"),
                Expr::symbol(x, "x") - Expr::symbol(y, "y"),
                Expr::symbol(x, "x") * Expr::symbol(y, "y"),
                Expr::symbol(x, "x") / Expr::symbol(y, "y"),
                Expr::real(left.clone()).sqrt(),
                Expr::real(right.clone()).ln(),
                Expr::real(left.clone()).sin(),
                Expr::real(right.clone()).cos(),
            ];
            for expression in expressions {
                let _ = expression.structural_facts();
                assert!(expression.eval_real(&bindings).is_ok());
            }

            assert_eq!(
                (Expr::symbol(x, "x") + Expr::symbol(y, "y"))
                    .eval_real(&bindings)
                    .expect("bound expression"),
                left + right
            );

            let mut problem = Problem::default();
            problem.add_variable("x", left.clone());
            problem.add_variable("y", right.clone());
            problem.add_constraint(Constraint::equality(
                "pair sum",
                Expr::symbol(x, "x") + Expr::symbol(y, "y") - Expr::real(left + right),
            ));
            let mut context = EvaluationContext::default();
            context.bind(x, left.clone());
            context.bind(y, right.clone());
            let residuals = evaluate_residuals(&problem, &context).expect("bound problem");
            assert_eq!(residuals.len(), 1);
            assert_eq!(residuals[0].value, Real::zero());
        }
    }
});

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
