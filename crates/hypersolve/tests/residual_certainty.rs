use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::{Real, RealSign};
use hypersolve::{
    CandidateResidualBall, CertifiedCandidateStatus, Constraint, ConstraintKind, EvaluationContext,
    Expr, FailedConstraintStatus, IntervalBoxCertificationPackage, IntervalBoxCertificationStatus,
    Problem, certify_candidate_with_residual_balls, certify_interval_box_candidate,
    diagnose_failed_constraints_from_certification,
};

fn problem(kind: ConstraintKind, positive: bool, exact_control: bool) -> Problem {
    // The angle-addition identity is independently zero; the retained tiny
    // rational shift makes every equality/soft residual strictly nonzero.
    let zero = if exact_control {
        Real::zero()
    } else {
        (Real::e() + Real::one()).sin()
            - Real::e().sin() * Real::one().cos()
            - Real::e().cos() * Real::one().sin()
    };
    let tiny = Real::from(2).powi_i64(-2048).unwrap();
    let value = zero + if positive { tiny } else { -tiny };
    let mut problem = Problem::default();
    problem.add_constraint(Constraint {
        name: "nonzero residual".to_owned(),
        kind,
        residual: Expr::real(value),
        weight: Real::one(),
        active: true,
    });
    problem
}

#[test]
fn ball_summary_helpers_require_exact_or_filtered_evidence() {
    for certainty in [
        Certainty::Exact,
        Certainty::Filtered,
        Certainty::Approximate,
    ] {
        for sign in [RealSign::Negative, RealSign::Zero, RealSign::Positive] {
            for satisfied in [false, true] {
                let status = CertifiedCandidateStatus::BallCertified {
                    sign,
                    certainty,
                    satisfied,
                };
                let certified = certainty != Certainty::Approximate;
                assert_eq!(status.is_certified_satisfied(), certified && satisfied);
                assert_eq!(status.is_certified_violation(), certified && !satisfied);
                assert_eq!(status.is_unknown(), !certified);
                assert!(!status.is_lossy_adapter_only());
            }
        }
    }
}

#[test]
fn approximate_residual_receipts_never_become_exact_summary_claims() {
    for kind in [
        ConstraintKind::Equality,
        ConstraintKind::Soft,
        ConstraintKind::LessOrEqual,
        ConstraintKind::GreaterOrEqual,
    ] {
        for positive in [false, true] {
            for exact_control in [false, true] {
                for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                    let problem = problem(kind, positive, exact_control);
                    let report = certify_candidate_with_residual_balls(
                        &problem.analyze(),
                        &EvaluationContext::default(),
                        &[CandidateResidualBall {
                            active_row: 0,
                            radius: Real::zero(),
                        }],
                        policy,
                    );
                    assert_eq!(report.rows.len(), 1);
                    assert_eq!(report.rows[0].constraint_index, 0);
                    assert_eq!(report.rows[0].kind, kind);
                    assert!(report.rows[0].signed_residual.is_some());
                    assert_eq!(report.domain_failure_rows, 0);
                    assert_eq!(report.lossy_adapter_only_rows, 0);
                    if exact_control {
                        let satisfied = matches!(
                            (kind, positive),
                            (ConstraintKind::LessOrEqual, false)
                                | (ConstraintKind::GreaterOrEqual, true)
                        );
                        assert!(matches!(
                            report.rows[0].status,
                            CertifiedCandidateStatus::BallCertified {
                                certainty: Certainty::Exact | Certainty::Filtered,
                                ..
                            }
                        ));
                        assert_eq!(report.all_satisfied(), satisfied);
                        assert_eq!(report.has_certified_violation(), !satisfied);
                        assert_eq!(report.bounded_unknown_rows, 0);
                    } else {
                        if policy == PredicatePolicy::STRICT {
                            assert!(matches!(
                                report.rows[0].status,
                                CertifiedCandidateStatus::BoundedUnknown { .. }
                            ));
                        } else {
                            assert!(matches!(
                                report.rows[0].status,
                                CertifiedCandidateStatus::BallCertified {
                                    certainty: Certainty::Approximate,
                                    sign: RealSign::Zero,
                                    satisfied: true
                                }
                            ));
                        }
                        assert_eq!(report.certified_satisfied_rows, 0);
                        assert_eq!(report.certified_violation_rows, 0);
                        assert_eq!(report.bounded_unknown_rows, 1);
                        assert!(!report.all_satisfied());
                        assert!(!report.has_certified_violation());
                        assert!(report.has_only_uncertainty());
                    }
                }
            }
        }
    }
}

#[test]
fn interval_box_facades_preserve_residual_uncertainty() {
    for kind in [
        ConstraintKind::Equality,
        ConstraintKind::Soft,
        ConstraintKind::LessOrEqual,
        ConstraintKind::GreaterOrEqual,
    ] {
        for positive in [false, true] {
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                for package in [
                    IntervalBoxCertificationPackage::Affine,
                    IntervalBoxCertificationPackage::UnivariateQuadratic,
                    IntervalBoxCertificationPackage::MultivariateQuadratic,
                ] {
                    let problem = problem(kind, positive, false);
                    let report = certify_interval_box_candidate(
                        &problem.analyze(),
                        &EvaluationContext::default(),
                        &[],
                        package,
                        policy,
                    );
                    assert_eq!(report.status, IntervalBoxCertificationStatus::Unknown);
                    let inner = report.certification.as_ref().unwrap();
                    assert_eq!(inner.certified_satisfied_rows, 0);
                    assert_eq!(inner.certified_violation_rows, 0);
                    assert_eq!(inner.bounded_unknown_rows, 1);
                }
            }
        }
    }
}

#[test]
fn failed_constraint_diagnostics_retain_approximate_ball_receipts() {
    let problem = problem(ConstraintKind::Equality, true, false);
    let analysis = problem.analyze();
    let report = certify_candidate_with_residual_balls(
        &analysis,
        &EvaluationContext::default(),
        &[CandidateResidualBall {
            active_row: 0,
            radius: Real::zero(),
        }],
        PredicatePolicy::APPROXIMATE_512,
    );
    let original = report.rows[0].clone();
    let diagnostic = diagnose_failed_constraints_from_certification(&analysis, report, -512);
    assert_eq!(diagnostic.rows.len(), 1);
    assert_eq!(diagnostic.rows[0].constraint_index, 0);
    assert_eq!(
        diagnostic.rows[0].status,
        FailedConstraintStatus::BoundedUnknown
    );
    assert_eq!(diagnostic.rows[0].certification.as_ref(), Some(&original));
}
