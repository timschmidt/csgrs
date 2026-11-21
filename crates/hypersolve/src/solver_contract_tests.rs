use crate::jacobian::{FiniteDifferenceConfig, finite_difference_jacobian};
use crate::{
    Constraint, DenseLinearBackend, DomainCheckStatus, Expr, LinearAdapterPrecision, LinearBackend,
    LinearSolveError, Problem, SymbolId, UnivariateQuadraticAlphaStatus, certify_candidate,
    certify_candidate_domains, certify_univariate_quadratic_alpha, context_from_problem,
    evaluate_residuals,
};
use hyperlimit::PredicatePolicy;
use hyperreal::Real;

#[test]
fn numerical_proposal_domain_and_alpha_reports_preserve_their_distinct_claims() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::one());
    let x = Expr::symbol(SymbolId(0), "x");
    problem.add_constraint(Constraint::equality(
        "quadratic",
        x.clone().powi(2) - Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality(
        "logarithmic",
        x.clone().ln() - Expr::int(2).ln(),
    ));
    let mut context = context_from_problem(&problem);
    context.bind(SymbolId(0), Real::from(3));
    let bindings = context.bindings().clone();
    let generation = context.generation();
    let jacobian = finite_difference_jacobian(
        &problem,
        &context,
        FiniteDifferenceConfig { step: Real::one() },
    )
    .unwrap();
    assert_eq!(jacobian.len(), 2);
    assert_eq!(jacobian[0], vec![7.0]); // (3+1)^2 - 3^2.
    assert!(jacobian[1][0] > 0.0);
    let residuals = evaluate_residuals(&problem, &context)
        .unwrap()
        .into_iter()
        .map(|row| row.weighted_dense_solver_estimate.unwrap())
        .collect::<Vec<_>>();
    assert_eq!(residuals[0], 5.0);
    let backend = DenseLinearBackend;
    assert_eq!(
        backend
            .solve_damped_normal(&[vec![f64::NAN]], &[1.0], 0.0)
            .unwrap_err(),
        LinearSolveError::NonFiniteInput
    );
    let (step, linear) = backend
        .solve_damped_normal(&jacobian, &residuals, 0.0)
        .unwrap();
    assert_eq!(linear.precision(), LinearAdapterPrecision::LossyF64);
    assert!(linear.is_lossy());
    assert_eq!(step.len(), 1);
    assert!(step[0].is_finite() && step[0] > -1.0 && step[0] < -0.5);
    assert_eq!(context.bindings(), &bindings);
    assert_eq!(context.generation(), generation);

    // Exact import preserves the proposed binary value. It does not turn the
    // numerical finite difference or the proposed solution into an exact proof.
    let proposed = Real::from(3) + Real::try_from(step[0]).unwrap();
    context.bind(SymbolId(0), proposed);
    let domain = certify_candidate_domains(&problem, &context, PredicatePolicy::STRICT);
    assert_eq!(domain.checks.len(), 2);
    assert_eq!(
        domain
            .checks
            .iter()
            .map(|check| (check.constraint_index, check.expression_path.as_str()))
            .collect::<Vec<_>>(),
        vec![(1, "$.left.arg"), (1, "$.right.arg")]
    );
    assert!(domain.all_checks_certified_valid());
    let alpha =
        certify_univariate_quadratic_alpha(&problem.analyze(), &context, PredicatePolicy::STRICT);
    assert_eq!(alpha.examined_rows, 1);
    assert_eq!(
        alpha.rows[0].status,
        UnivariateQuadraticAlphaStatus::CertifiedSmaleAlphaBound
    );
    let replay = certify_candidate(&problem.analyze(), &context);
    assert!(!replay.all_satisfied());
    assert!(replay.has_certified_violation());

    context.bind(SymbolId(0), Real::from(2));
    assert!(certify_candidate(&problem.analyze(), &context).all_satisfied());
    let exact =
        certify_univariate_quadratic_alpha(&problem.analyze(), &context, PredicatePolicy::STRICT);
    assert_eq!(
        exact.rows[0].status,
        UnivariateQuadraticAlphaStatus::ExactSimpleRoot
    );

    // Consume the same exact center in a near-root problem and an unresolved
    // domain. A basin certificate and an approximate domain answer together
    // still do not establish an exact root of the new model.
    let zero = crate::test_support::terminal_zero();
    let mut qualified = Problem::default();
    qualified.add_variable("x", Real::one());
    qualified.add_constraint(Constraint::equality(
        "near quadratic",
        x.powi(2) - Expr::int(4) + Expr::real(zero.clone() + Real::from(2).powi_i64(-600).unwrap()),
    ));
    qualified.add_constraint(Constraint::equality(
        "qualified domain",
        Expr::real(zero).sqrt(),
    ));
    let bindings = context.bindings().clone();
    let generation = context.generation();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let alpha = certify_univariate_quadratic_alpha(&qualified.analyze(), &context, policy);
        assert_eq!(alpha.examined_rows, 1);
        assert_eq!(
            alpha.rows[0].status,
            UnivariateQuadraticAlphaStatus::CertifiedSmaleAlphaBound
        );
        let domain = certify_candidate_domains(&qualified, &context, policy);
        assert_eq!(domain.checks.len(), 1);
        assert!(!domain.all_checks_certified_valid());
        assert!(!domain.has_certified_invalid_domain());
        assert_eq!(
            domain.approximate_checks(),
            usize::from(policy == PredicatePolicy::APPROXIMATE_512)
        );
        assert_eq!(
            domain.checks[0].status,
            if policy == PredicatePolicy::STRICT {
                DomainCheckStatus::Unknown
            } else {
                DomainCheckStatus::Approximate {
                    valid: true,
                    stage: hyperlimit::Escalation::Refined,
                }
            }
        );
        // Alpha and domain reports qualify different claims. Consuming their
        // shared exact context must retain residual-ball certainty, including
        // after changing policy back to STRICT with warmed proof caches.
        let residual = crate::certify_candidate_with_residual_balls(
            &qualified.analyze(),
            &context,
            &[crate::CandidateResidualBall {
                active_row: 0,
                radius: Real::zero(),
            }],
            policy,
        );
        assert_eq!(residual.rows.len(), 2);
        let row = residual.rows[0].clone();
        assert_eq!(row.constraint_index, 0);
        assert!(row.status.is_unknown());
        assert!(!row.status.is_certified_satisfied());
        assert!(!row.status.is_certified_violation());
        assert!(!residual.all_satisfied());
        assert!(!residual.has_certified_violation());
        if policy == PredicatePolicy::APPROXIMATE_512 {
            assert!(matches!(
                row.status,
                crate::CertifiedCandidateStatus::BallCertified {
                    certainty: hyperlimit::Certainty::Approximate,
                    sign: hyperreal::RealSign::Zero,
                    satisfied: true,
                }
            ));
        } else {
            assert!(matches!(
                row.status,
                crate::CertifiedCandidateStatus::BoundedUnknown { .. }
            ));
        }
        let diagnostic = crate::diagnose_failed_constraints_from_certification(
            &qualified.analyze(),
            residual,
            -512,
        );
        let diagnosed = diagnostic
            .rows
            .iter()
            .find(|entry| entry.constraint_index == 0)
            .unwrap();
        assert_eq!(
            diagnosed.status,
            crate::FailedConstraintStatus::BoundedUnknown
        );
        assert_eq!(diagnosed.certification.as_ref(), Some(&row));
        assert_eq!(context.bindings(), &bindings);
        assert_eq!(context.generation(), generation);
    }
    assert!(!certify_candidate(&qualified.analyze(), &context).all_satisfied());
}
