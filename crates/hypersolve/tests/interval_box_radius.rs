use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::Real;
use hypersolve::{
    AffineIntervalError, Constraint, Expr, IntervalBoxCertificationPackage as Package,
    IntervalBoxCertificationStatus as Status, Problem, QuadraticIntervalError, SymbolId,
    VariableBall, certify_affine_interval_candidate, certify_interval_box_candidate,
    certify_multivariate_quadratic_interval_candidate, certify_quadratic_interval_candidate,
    context_from_problem,
};

// sin(e+1) - sin(e)cos(1) - cos(e)sin(1) = 0.
// The shift survives exactly and lies below the terminal precision.
fn hidden_negative() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        - Real::from(2).powi_i64(-2048).unwrap()
}

#[test]
fn terminal_zero_cannot_validate_a_negative_box_radius() {
    let mut problem = Problem::default();
    problem.add_variable("x", Real::zero());
    problem.add_constraint(Constraint::equality("zero", Expr::int(0)));
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    let symbol = SymbolId(0);
    let bounds = [VariableBall {
        symbol,
        radius: hidden_negative(),
    }];
    assert!(matches!(
        compare_reals(
            &bounds[0].radius,
            &Real::zero(),
            PredicatePolicy::APPROXIMATE_512
        ),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            certainty: Certainty::Approximate,
            ..
        }
    ));
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert_eq!(
            certify_affine_interval_candidate(&analysis, &context, &bounds, policy),
            Err(AffineIntervalError::UnknownVariableRadiusSign { symbol })
        );
        for result in [
            certify_quadratic_interval_candidate(&analysis, &context, &bounds, policy),
            certify_multivariate_quadratic_interval_candidate(&analysis, &context, &bounds, policy),
        ] {
            assert_eq!(
                result,
                Err(QuadraticIntervalError::UnknownVariableRadiusSign { symbol })
            );
        }
        for package in [
            Package::Affine,
            Package::UnivariateQuadratic,
            Package::MultivariateQuadratic,
        ] {
            let report =
                certify_interval_box_candidate(&analysis, &context, &bounds, package, policy);
            assert_eq!(report.status, Status::InvalidInput);
            assert!(report.certification.is_none());
            assert!(
                report
                    .message
                    .unwrap()
                    .contains("unknown variable radius sign")
            );
            // The same solver analysis and context remain usable after rejection.
            for radius in [0, 1] {
                let retry = certify_interval_box_candidate(
                    &analysis,
                    &context,
                    &[VariableBall {
                        symbol,
                        radius: Real::from(radius),
                    }],
                    package,
                    policy,
                );
                assert_eq!(retry.status, Status::Certified);
            }
        }
    }
}

#[test]
fn krawczyk_packages_validate_the_box_before_using_a_radius() {
    use hypersolve::{
        AffineKrawczykStatus, MultivariateQuadraticKrawczykStatus, QuadraticKrawczykStatus,
        certify_affine_krawczyk_box, certify_multivariate_quadratic_krawczyk_box,
        certify_univariate_quadratic_krawczyk_box,
    };
    let symbol = SymbolId(0);
    let x = Expr::anonymous_symbol(symbol);
    let mut affine = Problem::default();
    affine.add_variable("x", Real::zero());
    affine.add_constraint(Constraint::equality("x", x.clone()));
    let mut quadratic = affine.clone();
    quadratic.constraints[0].residual = x.clone() * x.clone() + x;
    let a = affine.analyze();
    let q = quadratic.analyze();
    let context = context_from_problem(&affine);
    let bounds = [VariableBall {
        symbol,
        radius: hidden_negative(),
    }];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let linear = certify_affine_krawczyk_box(&a, &context, &bounds, policy);
        let univariate = certify_univariate_quadratic_krawczyk_box(&q, &context, &bounds, policy);
        let multivariate =
            certify_multivariate_quadratic_krawczyk_box(&q, &context, &bounds, policy);
        assert_eq!(
            (
                linear.status,
                &univariate.rows[0].status,
                multivariate.status
            ),
            (
                AffineKrawczykStatus::UnknownVariableRadiusSign { symbol },
                &QuadraticKrawczykStatus::UnknownVariableRadiusSign { symbol },
                MultivariateQuadraticKrawczykStatus::UnknownVariableRadiusSign { symbol }
            )
        );
    }
}
