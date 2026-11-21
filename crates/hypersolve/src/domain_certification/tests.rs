use super::*;
use crate::{Constraint, Expr, context_from_problem};

fn boundary(case: usize, exact_control: bool) -> (Expr, bool) {
    let zero = if exact_control {
        Real::zero()
    } else {
        crate::test_support::terminal_zero()
    };
    let tiny = Real::from(2).powi_i64(-2048).unwrap();
    let (value, valid) = match case {
        0..=2 | 4 | 5 => (zero, false),
        3 => (zero, true),
        6 => (zero - Real::one(), true),
        7 | 8 => (zero + Real::one(), true),
        9 => (zero + Real::one(), false),
        10 => (zero + tiny, true),
        11 => (zero - tiny, false),
        _ => unreachable!(),
    };
    let x = Expr::real(value);
    let expression = match case {
        0 | 10 => Expr::int(1) / x,
        1 => x.powi(-1),
        2 => x.powi(0),
        3 | 11 => x.sqrt(),
        4 => x.log10(),
        5 => x.ln(),
        6 => x.asin(),
        7 => x.acos(),
        8 => x.acosh(),
        9 => x.atanh(),
        _ => unreachable!(),
    };
    (expression, valid)
}

#[test]
fn approximate_domain_answers_retain_receipts_through_policy_transitions() {
    for case in 0..12 {
        // Angle addition proves the shared symbolic expression is zero.
        // The last two operands are independently +2^-2048 and -2^-2048.
        let (expression, mathematical_validity) = boundary(case, false);
        let mut problem = Problem::default();
        problem.add_constraint(Constraint::equality("domain boundary", expression));
        let context = EvaluationContext::default();
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            #[cfg(feature = "dispatch-trace")]
            let report = hyperreal::dispatch_trace::with_recording(|| {
                certify_candidate_domains(&problem, &context, policy)
            });
            #[cfg(not(feature = "dispatch-trace"))]
            let report = certify_candidate_domains(&problem, &context, policy);
            #[cfg(feature = "dispatch-trace")]
            {
                let trace = hyperreal::dispatch_trace::take_trace();
                assert_eq!(
                    trace.operation_count("hyperlimit", "compare_reals"),
                    if matches!(case, 6 | 7 | 9) { 2 } else { 1 },
                    "case {case}, {policy:?}: {trace:?}"
                );
                assert_eq!(
                    trace.operation_count("hyperlimit", "approximate_real_sign"),
                    u64::from(policy == PredicatePolicy::APPROXIMATE_512),
                    "case {case}, {policy:?}: {trace:?}"
                );
            }
            assert_eq!(report.active_constraint_count, 1);
            assert_eq!(report.checks.len(), 1);
            assert_eq!(report.certified_valid_checks, 0);
            assert_eq!(report.certified_invalid_checks, 0);
            assert_eq!(report.evaluation_failure_checks, 0);
            assert!(!report.all_checks_certified_valid());
            assert!(!report.has_certified_invalid_domain());
            if policy == PredicatePolicy::STRICT {
                assert_eq!(report.checks[0].status, DomainCheckStatus::Unknown);
                assert_eq!(report.unknown_checks, 1);
                assert_eq!(report.approximate_checks(), 0);
            } else {
                let approximate_validity = if case >= 10 {
                    !mathematical_validity
                } else {
                    mathematical_validity
                };
                assert_eq!(
                    report.checks[0].status,
                    DomainCheckStatus::Approximate {
                        valid: approximate_validity,
                        stage: Escalation::Refined,
                    }
                );
                assert_eq!(report.unknown_checks, 0);
                assert_eq!(report.approximate_checks(), 1);
            }
            assert_eq!(context.generation(), 0);
            assert!(context.bindings().is_empty());
        }
    }
}

#[test]
fn exact_domain_boundaries_remain_certified_under_both_policies() {
    for case in 0..12 {
        let (expression, valid) = boundary(case, true);
        let mut problem = Problem::default();
        problem.add_constraint(Constraint::equality("exact boundary", expression));
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = certify_candidate_domains(&problem, &EvaluationContext::default(), policy);
            assert_eq!(report.checks.len(), 1);
            assert_eq!(
                report.checks[0].status,
                if valid {
                    DomainCheckStatus::CertifiedValid
                } else {
                    DomainCheckStatus::CertifiedInvalid
                }
            );
            assert_eq!(report.all_checks_certified_valid(), valid);
            assert_eq!(report.has_certified_invalid_domain(), !valid);
            assert_eq!(report.certified_valid_checks, usize::from(valid));
            assert_eq!(report.certified_invalid_checks, usize::from(!valid));
            assert_eq!(report.approximate_checks(), 0);
            assert_eq!(report.unknown_checks, 0);
            assert_eq!(report.evaluation_failure_checks, 0);
        }
    }
}

#[test]
fn nested_domain_reports_keep_source_order_failures_and_qualifications() {
    let mut problem = Problem::default();
    let variable = problem.add_variable("positive", Real::from(4));
    problem.add_constraint(Constraint::equality(
        "bound symbol",
        Expr::symbol(SymbolId(variable.0), "positive").sqrt(),
    ));
    problem.add_constraint(Constraint::equality(
        "missing symbol",
        Expr::symbol(SymbolId(9), "missing").sqrt(),
    ));
    problem.add_constraint(Constraint::equality(
        "nested division",
        (Expr::int(1) / Expr::int(0)).sqrt(),
    ));
    let mut inactive = Constraint::equality("inactive", Expr::int(-1).ln());
    inactive.active = false;
    problem.add_constraint(inactive);
    problem.add_constraint(Constraint::equality(
        "qualified boundary",
        Expr::real(crate::test_support::terminal_zero()).sqrt(),
    ));
    let context = context_from_problem(&problem);
    let generation = context.generation();
    let bindings = context.bindings().clone();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let report = certify_candidate_domains(&problem, &context, policy);
        assert_eq!(report.active_constraint_count, 4);
        assert_eq!(report.checks.len(), 5);
        assert_eq!(report.certified_valid_checks, 1);
        assert_eq!(report.certified_invalid_checks, 1);
        assert_eq!(report.evaluation_failure_checks, 2);
        assert_eq!(
            report.unknown_checks,
            usize::from(policy == PredicatePolicy::STRICT)
        );
        assert_eq!(
            report.approximate_checks(),
            usize::from(policy == PredicatePolicy::APPROXIMATE_512)
        );
        assert!(!report.all_checks_certified_valid());
        assert!(report.has_certified_invalid_domain());
        assert_eq!(
            report
                .checks
                .iter()
                .map(|check| (check.constraint_index, check.expression_path.as_str()))
                .collect::<Vec<_>>(),
            vec![
                (0, "$.arg"),
                (1, "$.arg"),
                (2, "$.arg.right"),
                (2, "$.arg"),
                (4, "$.arg")
            ]
        );
        assert_eq!(
            report.checks[1].status,
            DomainCheckStatus::UnboundSymbol {
                symbol: SymbolId(9)
            }
        );
        assert!(matches!(
            report.checks[3].status,
            DomainCheckStatus::EvaluationFailure { .. }
        ));
        assert_eq!(context.generation(), generation);
        assert_eq!(context.bindings(), &bindings);
    }
}
