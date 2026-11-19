use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, Problem, SymbolId, UnivariateQuadraticAlphaStatus as Status,
    certify_univariate_quadratic_alpha, context_from_problem,
};

fn classify(linear: Real, constant: Real, policy: PredicatePolicy) -> Status {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::zero());
    let x = Expr::symbol(SymbolId(0), "x");
    problem.add_constraint(Constraint::equality(
        "quadratic",
        x.clone().powi(2) + Expr::real(linear) * x + Expr::real(constant),
    ));
    let report = certify_univariate_quadratic_alpha(
        &problem.analyze(),
        &context_from_problem(&problem),
        policy,
    );
    assert_eq!(report.examined_rows, 1);
    assert_eq!(
        report.certified_rows,
        usize::from(report.rows[0].status.is_certified())
    );
    report.rows.into_iter().next().unwrap().status
}

#[test]
fn exact_statuses_require_proved_zero_and_nonzero_values() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let t = Real::e();
        // sin(2t) = 2 sin(t) cos(t), independently of either predicate policy.
        let zero = (Real::from(2) * &t).sin() - Real::from(2) * t.clone().sin() * t.cos();
        let tiny = Real::from(2).powi((-600).into()).unwrap();
        assert_eq!(
            classify(zero.clone(), Real::zero(), policy),
            Status::Undecided
        );
        assert_eq!(
            classify(Real::zero(), zero.clone(), policy),
            Status::Undecided
        );
        assert_eq!(
            classify(Real::one(), zero + tiny, policy),
            Status::CertifiedSmaleAlphaBound
        );
        // At x=0 for x² + 4x + 2, both sides of the test equal 16.
        assert_eq!(
            classify(Real::from(4), Real::from(2), policy),
            Status::CertifiedSmaleAlphaBound
        );
        assert_eq!(
            classify(Real::one(), Real::zero(), policy),
            Status::ExactSimpleRoot
        );
        assert_eq!(
            classify(Real::zero(), Real::zero(), policy),
            Status::ExactMultipleRoot
        );
        assert_eq!(
            classify(Real::zero(), Real::one(), policy),
            Status::ZeroDerivative
        );
        assert_eq!(
            classify(Real::one(), Real::one(), policy),
            Status::BoundFailed
        );
    }
}
