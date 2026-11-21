use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    ActiveSetRowStatus, CandidateResidualBall, Constraint, ConstraintKind, Expr, Problem, SymbolId,
    audit_active_mask, certify_candidate, certify_candidate_with_residual_balls,
    context_from_problem,
};

#[test]
fn strict_and_closed_rows_replay_current_bindings_and_active_masks() {
    for (kind, strict) in [
        (ConstraintKind::GreaterOrEqual, false),
        (ConstraintKind::GreaterThan, true),
    ] {
        let mut problem = Problem::default();
        let x = problem.add_variable("x", Real::one());
        let mut constraint =
            Constraint::equality("half-line", Expr::anonymous_symbol(SymbolId(x.0)));
        constraint.kind = kind;
        problem.add_constraint(constraint);
        let analysis = problem.analyze();
        for value in [-3, 0, 3] {
            let mut context = context_from_problem(&problem);
            context.bind(SymbolId(x.0), Real::from(value));
            let inside = value > 0 || (!strict && value == 0);
            let report = certify_candidate(&analysis, &context);
            assert_eq!(report.all_satisfied(), inside);
            assert_eq!(report.has_certified_violation(), !inside);
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let ball = certify_candidate_with_residual_balls(
                    &analysis,
                    &context,
                    &[CandidateResidualBall {
                        active_row: 0,
                        radius: Real::zero(),
                    }],
                    policy,
                );
                assert_eq!(ball.all_satisfied(), inside);
                assert_eq!(ball.has_certified_violation(), !inside);
            }
            for active in [false, true] {
                let audit = audit_active_mask(&analysis, &context, &[active], Default::default());
                assert_eq!(
                    matches!(
                        audit.rows[0].status,
                        ActiveSetRowStatus::CertifiedViolation { .. }
                    ),
                    !inside
                );
            }
        }
    }
}

#[test]
fn point_ball_cannot_turn_an_unresolved_strict_sign_into_a_violation() {
    let positive = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        + Real::from(2).powi_i64(-2048).unwrap();
    for (value, inside) in [(positive.clone(), true), (-positive, false)] {
        let mut problem = Problem::default();
        let mut row = Constraint::equality("positive", Expr::real(value));
        row.kind = ConstraintKind::GreaterThan;
        problem.add_constraint(row);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let ball = certify_candidate_with_residual_balls(
                &problem.analyze(),
                &context_from_problem(&problem),
                &[CandidateResidualBall {
                    active_row: 0,
                    radius: Real::zero(),
                }],
                policy,
            );
            assert!(
                ball.has_only_uncertainty()
                    || (ball.all_satisfied() == inside && ball.has_certified_violation() != inside)
            );
        }
    }
}
