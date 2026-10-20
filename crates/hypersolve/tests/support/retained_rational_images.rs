use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::bivariate_swap_parameters;
use hypersolve::*;

fn p(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn base() -> AlgebraicRootRepresentation {
    let mut root = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![(-2).into(), 0.into(), 1.into()],
        interval: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    assert!(root.is_valid());
    root
}

fn config() -> AlgebraicFiberPolynomialImageProjectionConfig {
    AlgebraicFiberPolynomialImageProjectionConfig {
        max_fiber_degree: 8,
        max_retained_degree: 8,
        max_image_degree_bound: 8,
    }
}

// Exact independent coefficient oracle, up to a nonzero rational unit.
fn proportional(actual: &BivariatePolynomial, expected: &BivariatePolynomial) {
    let get = |p: &BivariatePolynomial, i: usize, j: usize| {
        p.coefficients
            .get(i)
            .and_then(|row| row.get(j))
            .cloned()
            .unwrap_or_else(Real::zero)
    };
    let (i, j, unit) = expected
        .coefficients
        .iter()
        .enumerate()
        .find_map(|(i, row)| {
            row.iter()
                .enumerate()
                .find(|(_, x)| !x.definitely_zero())
                .map(|(j, x)| (i, j, x.clone()))
        })
        .unwrap();
    let factor = (get(actual, i, j) / unit).unwrap();
    assert!(!factor.exact_rational_normal_form().unwrap().is_zero());
    let width = actual
        .coefficients
        .iter()
        .chain(&expected.coefficients)
        .map(Vec::len)
        .max()
        .unwrap();
    for i in 0..actual.coefficients.len().max(expected.coefficients.len()) {
        for j in 0..width {
            assert!(
                (get(actual, i, j) - &factor * get(expected, i, j))
                    .exact_rational_normal_form()
                    .unwrap()
                    .is_zero()
            );
        }
    }
}

pub fn mixed_images_keep_the_base_field_on_both_axes() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (f, n, d, expected) in [
            // y^3=x, z=x+y => z^3-3*x*z^2+6*z-3*x, x^2=2.
            (
                p(&[&[0, 0, 0, 1], &[-1]]),
                p(&[&[0, 1], &[1]]),
                p(&[&[1]]),
                p(&[&[0, 6, 0, 1], &[-3, 0, -3]]),
            ),
            // y^2=3, z=(x+y)/(1+y) => -2*z^2+(6-2*x)*z-1.
            (
                p(&[&[-3, 0, 1]]),
                p(&[&[0, 1], &[1]]),
                p(&[&[1, 1]]),
                p(&[&[-1, 6, -2], &[0, -2]]),
            ),
            // The two y roots have the same image y^2=x.
            (
                p(&[&[0, 0, 1], &[-1]]),
                p(&[&[0, 0, 1]]),
                p(&[&[1]]),
                p(&[&[2, 0, 1], &[0, -2]]),
            ),
        ] {
            for swap in [false, true] {
                let oriented = |value: &BivariatePolynomial| {
                    if swap {
                        bivariate_swap_parameters(value)
                    } else {
                        value.clone()
                    }
                };
                let report = project_algebraic_fiber_rational_image_relation(
                    &oriented(&f),
                    if swap {
                        CurveResultantParameter::Second
                    } else {
                        CurveResultantParameter::First
                    },
                    &oriented(&n),
                    &oriented(&d),
                    &base(),
                    config(),
                    policy,
                );
                assert_eq!(
                    report.status,
                    AlgebraicFiberPolynomialImageProjectionStatus::Constructed
                );
                assert_eq!(report.certainty, Certainty::Exact);
                assert!(report.coefficients.is_empty(), "no global image norm");
                assert!(report.identically_zero_fiber_factor.is_none());
                proportional(report.retained_relation.as_ref().unwrap(), &expected);
            }
        }
    }
}

pub fn source_poles_and_zero_image_components_remain_explicit() {
    // F=y(y^2-x); N=y(x+y), D=y. The undefined y=0 source cannot
    // contribute the entire image line after saturation.
    let report = project_algebraic_fiber_rational_image_relation(
        &p(&[&[0, 0, 0, 1], &[0, -1]]),
        CurveResultantParameter::First,
        &p(&[&[0, 0, 1], &[0, 1]]),
        &p(&[&[0, 1]]),
        &base(),
        config(),
        PredicatePolicy::STRICT,
    );
    assert_eq!(
        report.status,
        AlgebraicFiberPolynomialImageProjectionStatus::Constructed
    );
    proportional(
        report.retained_relation.as_ref().unwrap(),
        // The norm retains Norm(y)=-x, a nonzero field unit at sqrt(2):
        // -x*((z-x)^2-x) = 2+4*z-2*x-x*z^2.
        &p(&[&[2, 4], &[-2, 0, -1]]),
    );
    proportional(
        report.identically_zero_fiber_factor.as_ref().unwrap(),
        &p(&[&[0, 1]]),
    );
    let all_zero = project_algebraic_fiber_rational_image_relation(
        &p(&[&[0, 0, 1], &[-1]]),
        CurveResultantParameter::First,
        &p(&[&[0]]),
        &p(&[&[0]]),
        &base(),
        config(),
        PredicatePolicy::STRICT,
    );
    assert_eq!(
        all_zero.status,
        AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation
    );
    assert!(all_zero.retained_relation.is_none());
}

pub fn rational_image_limits_and_budget_recovery() {
    let f = p(&[&[0, 0, 0, 1], &[-1]]);
    let n = p(&[&[0, 1], &[1]]);
    let d = p(&[&[1]]);
    let root = base();
    let project = |root: &AlgebraicRootRepresentation, config| {
        project_algebraic_fiber_rational_image_relation(
            &f,
            CurveResultantParameter::First,
            &n,
            &d,
            root,
            config,
            PredicatePolicy::STRICT,
        )
    };
    let expected = project(&root, config());
    for limit in [
        AlgebraicFiberPolynomialImageProjectionConfig {
            max_fiber_degree: 2,
            ..config()
        },
        AlgebraicFiberPolynomialImageProjectionConfig {
            max_retained_degree: 1,
            ..config()
        },
        AlgebraicFiberPolynomialImageProjectionConfig {
            max_image_degree_bound: 2,
            ..config()
        },
    ] {
        let report = project(&root, limit);
        assert_eq!(
            report.status,
            AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded
        );
        assert!(report.retained_relation.is_none());
        assert!(report.coefficients.is_empty());
    }
    let mut bad = root.clone();
    bad.interval.exact_root = Some(Real::one());
    assert_eq!(
        project(&bad, config()).status,
        AlgebraicFiberPolynomialImageProjectionStatus::InvalidEvidence
    );
    let denied = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || {
            let report = project(&root, config());
            assert_eq!(
                report.status,
                AlgebraicFiberPolynomialImageProjectionStatus::BudgetExhausted
            );
            assert!(report.retained_relation.is_none());
            assert!(report.identically_zero_fiber_factor.is_none());
            assert!(report.coefficients.is_empty());
        },
    );
    assert_eq!(
        denied.result.unwrap_err().kind,
        work_budget::WorkKind::AlgebraicPromotion
    );
    let retry = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 4096,
            promotions: 1,
        },
        || project(&root, config()),
    );
    assert_eq!(retry.result.unwrap(), expected);
    assert_eq!(retry.usage.promotions, 1);
}
