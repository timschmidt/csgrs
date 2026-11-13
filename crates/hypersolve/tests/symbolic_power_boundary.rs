use hyperreal::{Problem, Real};
use hypersolve::{Expr, ExprEvalError, SymbolId};
use std::collections::HashMap;

// Independent homogeneity accounting uses a wider exponent than Expr's storage.
// It never materializes x raised to an enormous integer. These witnesses are
// monomials on x != 0; zero factors are ignored only for this degree calculation,
// not for evaluation or domain validation.
fn degree(expression: &Expr, symbol: SymbolId) -> Option<i128> {
    match expression {
        Expr::Constant(value) => (value != &Real::zero()).then_some(0),
        Expr::Symbol(reference) => {
            assert_eq!(reference.id, symbol);
            Some(1)
        }
        Expr::Neg(value) => degree(value, symbol),
        Expr::Add(left, right) | Expr::Sub(left, right) => {
            match (degree(left, symbol), degree(right, symbol)) {
                (None, other) | (other, None) => other,
                (Some(a), Some(b)) => {
                    assert_eq!(a, b, "nonzero terms must have equal homogeneous degree");
                    Some(a)
                }
            }
        }
        Expr::Mul(left, right) => Some(degree(left, symbol)? + degree(right, symbol)?),
        Expr::Div(left, right) => Some(degree(left, symbol)? - degree(right, symbol).unwrap()),
        Expr::PowI(value, exponent) => Some(degree(value, symbol)? * i128::from(*exponent)),
        _ => panic!("not a monomial witness"),
    }
}

#[test]
fn power_rule_covers_both_integer_boundaries_and_repeated_derivatives() {
    let symbol = SymbolId(0);
    let bindings = HashMap::from([(symbol, Real::one())]);
    for exponent in [i64::MIN, i64::MIN + 1, -7, -2, -1, 0, 1, 2, 7, i64::MAX] {
        let mut expression = Expr::anonymous_symbol(symbol).powi(exponent);
        let mut coefficient = Real::one();
        for order in 1..=3 {
            expression = expression.derivative(symbol);
            coefficient *= Real::from(exponent) - Real::from(order - 1);
            let expected_degree =
                (coefficient != Real::zero()).then_some(i128::from(exponent) - i128::from(order));
            assert_eq!(degree(&expression, symbol), expected_degree);
            assert_eq!(expression.eval_real(&bindings).unwrap(), coefficient);
        }
    }
}

#[test]
fn minimum_power_chain_rule_preserves_nonzero_domain_and_failure() {
    let x = SymbolId(0);
    let y = SymbolId(1);
    let source = (Expr::int(2) * Expr::anonymous_symbol(x) - Expr::int(1)).powi(i64::MIN);
    let first = source.derivative(x);
    let second = first.derivative(x);
    let unrelated = source.derivative(y);
    let n = Real::from(i64::MIN);
    for (point, first_value) in [
        (0, -Real::from(2) * n.clone()),
        (1, Real::from(2) * n.clone()),
    ] {
        let bindings = HashMap::from([(x, Real::from(point))]);
        assert_eq!(source.eval_real(&bindings).unwrap(), Real::one());
        assert_eq!(first.eval_real(&bindings).unwrap(), first_value);
        assert_eq!(
            second.eval_real(&bindings).unwrap(),
            Real::from(4) * n.clone() * (n.clone() - Real::one())
        );
        assert_eq!(unrelated.eval_real(&bindings).unwrap(), Real::zero());
    }
    let singular = HashMap::from([(x, (Real::one() / Real::from(2)).unwrap())]);
    for expression in [&source, &first, &second, &unrelated] {
        assert!(matches!(
            expression.eval_real(&singular),
            Err(ExprEvalError::Hyperreal(Problem::NotANumber))
        ));
        assert!(
            matches!(expression.eval_real(&HashMap::new()), Err(ExprEvalError::UnboundSymbol(reference)) if reference.id == x)
        );
    }
}

#[test]
fn dense_jacobian_consumes_minimum_power_derivative() {
    use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_with_analysis};
    use hypersolve::{Constraint, Problem as SolverProblem, context_from_problem};

    let mut problem = SolverProblem::default();
    let variable = problem.add_variable("x", Real::one());
    problem.add_constraint(Constraint::equality(
        "minimum power",
        Expr::anonymous_symbol(variable.into()).powi(i64::MIN),
    ));
    let context = context_from_problem(&problem);
    let analysis = problem.analyze();
    // This integer is exactly representable in binary64; these APIs remain
    // proposal adapters, not interval/Jacobian certificates.
    let expected = vec![vec![-9_223_372_036_854_775_808.0]];
    assert_eq!(symbolic_jacobian(&problem, &context).unwrap(), expected);
    assert_eq!(
        symbolic_jacobian_with_analysis(&analysis, &context).unwrap(),
        expected
    );
}

#[test]
fn unrepresentable_dense_jacobian_is_a_typed_failure() {
    use hypersolve::eval::EvalError;
    use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_with_analysis};
    use hypersolve::{Constraint, Problem as SolverProblem, context_from_problem};

    let mut problem = SolverProblem::default();
    let variable = problem.add_variable("x", Real::zero());
    let coefficient = Real::from(2).powi_i64(2048).unwrap();
    assert!(coefficient.to_f64_lossy().is_none());
    problem.add_constraint(Constraint::equality(
        "finite exact slope outside binary64",
        Expr::real(coefficient) * Expr::anonymous_symbol(variable.into()),
    ));
    let context = context_from_problem(&problem);
    let analysis = problem.analyze();
    for result in [
        symbolic_jacobian(&problem, &context),
        symbolic_jacobian_with_analysis(&analysis, &context),
    ] {
        assert!(
            matches!(result, Err(EvalError::DenseSolverEstimateUnavailable(_))),
            "{result:?}"
        );
    }
}

#[test]
fn identity_power_derivative_accepts_zero_and_retains_base_guards() {
    let symbol = SymbolId(0);
    let x = || Expr::anonymous_symbol(symbol);
    let derivative = x().powi(1).derivative(symbol);
    for point in [-2, 0, 3] {
        let bindings = HashMap::from([(symbol, Real::from(point))]);
        assert_eq!(derivative.eval_real(&bindings).unwrap(), Real::one());
    }
    let guarded = x().ln().powi(1).derivative(symbol);
    for point in [1, 2] {
        let bindings = HashMap::from([(symbol, Real::from(point))]);
        assert_eq!(
            guarded.eval_real(&bindings).unwrap(),
            (Real::one() / Real::from(point)).unwrap()
        );
    }
    for point in [-1, 0] {
        let bindings = HashMap::from([(symbol, Real::from(point))]);
        assert!(matches!(
            guarded.eval_real(&bindings),
            Err(ExprEvalError::Hyperreal(_))
        ));
    }
    // Identity powers must not turn an actually singular derivative into zero.
    let singular = x().sqrt().powi(1).derivative(symbol);
    assert!(matches!(
        singular.eval_real(&HashMap::from([(symbol, Real::zero())])),
        Err(ExprEvalError::Hyperreal(_))
    ));
}
