use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, IntervalBoxCertificationPackage as Package,
    IntervalBoxCertificationStatus as Status, Problem, SymbolId, VariableBall,
    certify_interval_box_candidate, context_from_problem,
};

fn cubic_problem(center: i64) -> Problem {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::from(center));
    problem.add_variable("y", Real::from(4));
    let x = Expr::anonymous_symbol(SymbolId(0));
    problem.add_constraint(Constraint::equality("x cubed", x.clone() * x.clone() * x));
    problem
}

fn ball(symbol: u32, radius: i64) -> VariableBall {
    VariableBall {
        symbol: SymbolId(symbol),
        radius: Real::from(radius),
    }
}

#[test]
fn cubic_center_cannot_certify_a_box_that_contains_both_outcomes() {
    // Both [-1,1] and [0,2] contain x=0 (satisfied) and x=1 (violated).
    for center in [0, 1] {
        let problem = cubic_problem(center);
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        for package in [
            Package::Affine,
            Package::UnivariateQuadratic,
            Package::MultivariateQuadratic,
        ] {
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let report = certify_interval_box_candidate(
                    &analysis,
                    &context,
                    &[ball(0, 1)],
                    package,
                    policy,
                );
                let point = report.certification.unwrap();
                assert_eq!(point.all_satisfied(), center == 0);
                assert_eq!(point.has_certified_violation(), center == 1);
                assert_eq!(
                    report.status,
                    Status::Unknown,
                    "center={center}, {package:?}"
                );
            }
        }
    }
}

#[test]
fn constant_dependencies_and_singleton_coordinates_transport_point_evidence() {
    for center in [0, 1] {
        let problem = cubic_problem(center);
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        for radii in [
            vec![],
            vec![ball(0, 0)],
            vec![ball(1, 4)],
            vec![ball(0, 0), ball(1, 4)],
        ] {
            let expected = if center == 0 {
                Status::Certified
            } else {
                Status::Violation
            };
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let report = certify_interval_box_candidate(
                    &analysis,
                    &context,
                    &radii,
                    Package::Affine,
                    policy,
                );
                assert_eq!(report.status, expected);
            }
        }
    }
}

#[test]
fn only_a_whole_box_violation_decides_a_mixed_report() {
    for center in [0, 1] {
        for y_target in [4, 7] {
            for reversed in [false, true] {
                let mut problem = cubic_problem(center);
                let y = Expr::anonymous_symbol(SymbolId(1));
                problem.add_constraint(Constraint::equality(
                    "y equals target",
                    y - Expr::int(y_target),
                ));
                if reversed {
                    problem.constraints.reverse();
                }
                let mut inactive = Constraint::equality("inactive", Expr::int(3));
                inactive.active = false;
                problem.constraints.insert(0, inactive);
                let analysis = problem.analyze();
                let context = context_from_problem(&problem);
                for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                    let report = certify_interval_box_candidate(
                        &analysis,
                        &context,
                        &[ball(0, 1)],
                        Package::Affine,
                        policy,
                    );
                    assert_eq!(
                        report.status,
                        if y_target == 7 {
                            Status::Violation
                        } else {
                            Status::Unknown
                        }
                    );
                    assert_eq!(report.certification.unwrap().rows.len(), 2);
                }
            }
        }
    }
}

#[test]
fn external_context_symbols_and_repeated_ball_bindings_follow_replay() {
    let symbol = SymbolId(37);
    let x = Expr::anonymous_symbol(symbol);
    let mut problem = Problem::default();
    problem.add_constraint(Constraint::equality(
        "bound parameter cubed",
        x.clone() * x.clone() * x,
    ));
    let mut context = context_from_problem(&problem);
    context.bind(symbol, Real::one());
    let analysis = problem.analyze();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (radii, expected) in [
            (vec![ball(37, 0), ball(37, 1)], Status::Unknown),
            (vec![ball(37, 1), ball(37, 0)], Status::Violation),
            (vec![ball(37, -1), ball(37, 0)], Status::InvalidInput),
        ] {
            let report = certify_interval_box_candidate(
                &analysis,
                &context,
                &radii,
                Package::Affine,
                policy,
            );
            assert_eq!(report.status, expected);
            assert_eq!(report.variable_balls, radii);
        }
    }
}

// sin(e+1) - sin(e)cos(1) - cos(e)sin(1) = 0.
// The shift survives exactly and lies below the terminal precision.
fn hidden_negative() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        - Real::from(2).powi_i64(-2048).unwrap()
}

#[test]
fn approximate_ball_receipt_is_preserved_without_a_box_certificate() {
    let mut problem = Problem::default();
    problem.add_constraint(Constraint::equality(
        "nonzero constant",
        Expr::real(hidden_negative()),
    ));
    let report = certify_interval_box_candidate(
        &problem.analyze(),
        &context_from_problem(&problem),
        &[],
        Package::Affine,
        PredicatePolicy::APPROXIMATE_512,
    );
    let replay = report.certification.unwrap();
    assert!(matches!(
        replay.rows[0].status,
        hypersolve::CertifiedCandidateStatus::BallCertified {
            certainty: hyperlimit::Certainty::Approximate,
            ..
        }
    ));
    assert_eq!(report.status, Status::Unknown);
    assert_eq!(replay.certified_satisfied_rows, 0);
    assert_eq!(replay.certified_violation_rows, 0);
    assert_eq!(replay.bounded_unknown_rows, 1);
    assert!(replay.has_only_uncertainty());
}

#[test]
fn satisfaction_and_violation_counts_require_certified_ball_receipts() {
    use hyperlimit::Certainty;
    use hyperreal::RealSign;
    use hypersolve::CertifiedCandidateStatus;
    for certainty in [
        Certainty::Exact,
        Certainty::Filtered,
        Certainty::Approximate,
    ] {
        for satisfied in [false, true] {
            let status = CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Positive,
                certainty,
                satisfied,
            };
            let certified = certainty != Certainty::Approximate;
            assert_eq!(status.is_certified_satisfied(), certified && satisfied);
            assert_eq!(status.is_certified_violation(), certified && !satisfied);
            assert_eq!(status.is_unknown(), !certified);
        }
    }
}
