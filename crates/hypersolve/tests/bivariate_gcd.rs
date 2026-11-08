//! Shared factor authority for guarded selected-curve consumers.
use hyperreal::Real;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::bivariate_multiply;
use hypersolve::bivariate_topology::{
    BivariateTopologyConfig, BivariateTopologyError, bivariate_polynomial_gcd,
};
use hypersolve::work_budget::{WorkBudget, with_budget};

fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

#[test]
fn common_supports_keep_multiplicities_and_nonaxis_factors() {
    for common in [
        p(&[&[0, 0, -1], &[1]]),
        p(&[&[0, 0, 0, 0, 0, 0, 0, 0, 0, -1], &[1]]),
        bivariate_multiply(&p(&[&[-1], &[1]]), &p(&[&[-2, 1]])),
    ] {
        for common in [common.clone(), bivariate_multiply(&common, &common)] {
            let common = hypersolve::exact_factor::bivariate_trim_exact(common).unwrap();
            let first = bivariate_multiply(&common, &p(&[&[-3], &[1]]));
            let second = bivariate_multiply(&common, &p(&[&[-4, 1]]));
            let found = bivariate_polynomial_gcd(&first, &second, Default::default()).unwrap();
            assert_eq!(found, common);
            for input in [&first, &second] {
                assert!(hypersolve::divide_bivariate_polynomial_exact(input, &found).is_some());
            }
            assert_eq!(
                bivariate_polynomial_gcd(&second, &first, Default::default()).unwrap(),
                found
            );
        }
    }
}

#[test]
fn zero_inputs_and_coprime_supports_have_exact_gcds() {
    let zero = p(&[]);
    assert_eq!(
        bivariate_polynomial_gcd(&zero, &zero, Default::default()).unwrap(),
        zero
    );
    let source = p(&[&[0, 0, -1], &[1]]);
    for (a, b) in [(&source, &zero), (&zero, &source)] {
        assert_eq!(
            bivariate_polynomial_gcd(a, b, Default::default()).unwrap(),
            source
        );
    }
    assert_eq!(
        bivariate_polynomial_gcd(&source, &p(&[&[-2, 1]]), Default::default()).unwrap(),
        p(&[&[1]])
    );
}

#[test]
fn factor_budget_failure_does_not_poison_a_larger_retry() {
    let common = p(&[&[0, 0, -1], &[1]]);
    let first = bivariate_multiply(&common, &p(&[&[-3], &[1]]));
    let second = bivariate_multiply(&common, &p(&[&[-4, 1]]));
    let denied = with_budget(WorkBudget::ZERO, || {
        assert_eq!(
            bivariate_polynomial_gcd(&first, &second, Default::default()),
            Err(BivariateTopologyError::BudgetExhausted)
        );
    });
    assert!(denied.result.is_err());
    assert_eq!(
        bivariate_polynomial_gcd(
            &first,
            &second,
            BivariateTopologyConfig {
                max_projection_degree: 0,
                ..Default::default()
            }
        ),
        Err(BivariateTopologyError::DegreeLimit)
    );
    let retry = with_budget(
        WorkBudget {
            promotions: 100_000,
            refinements: 1_000_000,
        },
        || bivariate_polynomial_gcd(&first, &second, Default::default()).unwrap(),
    );
    assert_eq!(retry.result.unwrap(), common);
    assert!(retry.usage.promotions > 0);
}
