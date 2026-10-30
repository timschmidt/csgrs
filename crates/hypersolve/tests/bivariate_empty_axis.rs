use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyError, decompose_bivariate_curve,
};

fn domain() -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [(-1_i32).into(), 1_i32.into()],
        second: [(-2_i32).into(), 2_i32.into()],
    }
}

fn assert_empty(equation: BivariatePolynomial) {
    let rectangle = domain();
    let topology = decompose_bivariate_curve(&equation, &rectangle, Default::default())
        .expect("the equation has no real zero in the closed rectangle");
    assert_eq!(topology.equation(), &equation);
    assert_eq!(topology.domain(), &rectangle);
    assert!(
        topology
            .events()
            .iter()
            .all(|event| event.roots().is_empty())
    );
    assert!(topology.slabs().iter().all(|slab| slab.arcs().is_empty()));
    for x in [-1_i32, 0, 1] {
        assert!(
            topology
                .rational_fiber(&x.into(), Default::default())
                .unwrap()
                .roots()
                .is_empty()
        );
    }
}

#[test]
fn positive_axis_polynomial_has_empty_topology() {
    // x^2 + 1 >= 1 for every real x, independently of the solver.
    assert_empty(BivariatePolynomial::new(vec![
        vec![1_i32.into()],
        vec![],
        vec![1_i32.into()],
    ]));
}

#[test]
fn excluded_axis_root_has_empty_topology() {
    // The sole zero of x - 3 is outside the closed x-domain [-1, 1].
    assert_empty(BivariatePolynomial::new(vec![
        vec![(-3_i32).into()],
        vec![1_i32.into()],
    ]));
}

#[test]
fn existing_constant_empty_and_live_vertical_outcomes_stay_distinct() {
    assert_empty(BivariatePolynomial::new(vec![vec![2_i32.into()]]));
    for offset in [-1_i32, 0, 1] {
        let equation = BivariatePolynomial::new(vec![vec![offset.into()], vec![1_i32.into()]]);
        assert_eq!(
            decompose_bivariate_curve(&equation, &domain(), Default::default()),
            Err(BivariateTopologyError::VerticalComponent)
        );
    }
}

#[test]
fn malformed_domain_is_not_hidden_by_empty_geometry() {
    let equation = BivariatePolynomial::new(vec![vec![2_i32.into()]]);
    let mut rectangle = domain();
    rectangle.first = [1_i32.into(), (-1_i32).into()];
    assert_eq!(
        decompose_bivariate_curve(&equation, &rectangle, Default::default()),
        Err(BivariateTopologyError::InvalidDomain)
    );
}

#[test]
fn repeated_and_higher_degree_axis_sources_keep_empty_domains() {
    // (x-3)^2, x^6+1 and x^2-2 have no zero in [-1, 1].
    for coefficients in [vec![9, -6, 1], vec![1, 0, 0, 0, 0, 0, 1], vec![-2, 0, 1]] {
        assert_empty(BivariatePolynomial::new(
            coefficients.into_iter().map(|c| vec![c.into()]).collect(),
        ));
    }
}

#[test]
fn algebraic_and_repeated_boundary_roots_remain_live_vertical_components() {
    let mut rectangle = domain();
    rectangle.first = [(-2_i32).into(), 2_i32.into()];
    let irrational =
        BivariatePolynomial::new(vec![vec![(-2_i32).into()], vec![], vec![1_i32.into()]]);
    assert_eq!(
        decompose_bivariate_curve(&irrational, &rectangle, Default::default()),
        Err(BivariateTopologyError::VerticalComponent)
    );
    let boundary = BivariatePolynomial::new(vec![
        vec![1_i32.into()],
        vec![(-2_i32).into()],
        vec![1_i32.into()],
    ]);
    assert_eq!(
        decompose_bivariate_curve(&boundary, &domain(), Default::default()),
        Err(BivariateTopologyError::VerticalComponent)
    );
}

#[test]
fn empty_axis_degree_and_work_refusals_allow_a_larger_retry() {
    use hypersolve::bivariate_topology::BivariateTopologyConfig;
    use hypersolve::work_budget::{self, WorkBudget};
    let equation = BivariatePolynomial::new(vec![vec![1_i32.into()], vec![], vec![1_i32.into()]]);
    for config in [
        BivariateTopologyConfig {
            max_axis_degree: 1,
            ..Default::default()
        },
        BivariateTopologyConfig {
            max_projection_degree: 2,
            ..Default::default()
        },
    ] {
        assert_eq!(
            decompose_bivariate_curve(&equation, &domain(), config),
            Err(BivariateTopologyError::DegreeLimit)
        );
    }
    let denied = work_budget::with_budget(WorkBudget::ZERO, || {
        let result = decompose_bivariate_curve(&equation, &domain(), Default::default());
        assert_eq!(result, Err(BivariateTopologyError::BudgetExhausted));
        result
    });
    assert!(denied.result.is_err());
    // This rational census requires no additional algebraic promotion. A small
    // budget that suffices must not be misreported as resource exhaustion.
    let small = work_budget::with_budget(
        WorkBudget {
            refinements: 100_000,
            promotions: 1,
        },
        || decompose_bivariate_curve(&equation, &domain(), Default::default()),
    );
    assert!(small.result.unwrap().unwrap().is_empty());
    let recovered = work_budget::with_budget(
        WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || decompose_bivariate_curve(&equation, &domain(), Default::default()),
    );
    assert!(recovered.result.unwrap().unwrap().is_empty());
    assert_empty(equation);
}
