//! Repeated defining factors keep their original authority and reduced geometry.
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_specialize_first};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateCurveTopology, BivariateTopologyConfig,
    decompose_bivariate_curve,
};
use hypersolve::{BivariatePolynomial, UnivariateSturmSequence};

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn power(source: &BivariatePolynomial, exponent: usize) -> BivariatePolynomial {
    (0..exponent).fold(polynomial(&[&[1]]), |value, _| {
        bivariate_multiply(&value, source)
    })
}
fn rectangle() -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-2).into(), 2.into()],
    }
}
fn config() -> BivariateTopologyConfig {
    BivariateTopologyConfig {
        max_axis_degree: 12,
        max_projection_degree: 256,
        ..Default::default()
    }
}
fn build(source: &BivariatePolynomial) -> BivariateCurveTopology {
    decompose_bivariate_curve(source, &rectangle(), config()).unwrap()
}
fn layout(topology: &BivariateCurveTopology) -> (Vec<usize>, Vec<Vec<(usize, usize)>>) {
    (
        topology
            .events()
            .iter()
            .map(|event| event.roots().len())
            .collect(),
        topology
            .slabs()
            .iter()
            .map(|slab| {
                slab.arcs()
                    .iter()
                    .map(|arc| (arc.left_root(), arc.right_root()))
                    .collect()
            })
            .collect(),
    )
}

fn verify_fibers(topology: &BivariateCurveTopology, support: &BivariatePolynomial) {
    // Independent univariate counts on the known reduced support validate
    // each reported selector and the complete closed fiber. Original repeated
    // coefficients and the graph incidence are checked separately.
    for x in [-2, -1, 0, 1, 2].map(Real::from) {
        let row = bivariate_specialize_first(support, &x);
        let sequence = UnivariateSturmSequence::new(&row, PredicatePolicy::STRICT).unwrap();
        let fiber = topology.rational_fiber(&x, config().fiber).unwrap();
        let expected = sequence
            .count_distinct_roots(&Real::from(-2), &Real::from(2), PredicatePolicy::STRICT)
            .unwrap()
            + usize::from(Real::eval_poly(&row, &Real::from(-2)).definitely_zero());
        assert_eq!(fiber.roots().len(), expected);
        for root in fiber.roots() {
            if let Some(value) = &root.exact_root {
                assert!(Real::eval_poly(&row, value).definitely_zero());
            } else {
                let count = sequence
                    .count_distinct_roots(&root.lower, &root.upper, PredicatePolicy::STRICT)
                    .unwrap()
                    + usize::from(Real::eval_poly(&row, &root.lower).definitely_zero());
                assert_eq!(count, 1);
            }
        }
    }
}

fn assert_polynomial_eq(first: &BivariatePolynomial, second: &BivariatePolynomial) {
    for i in 0..first.coefficients.len().max(second.coefficients.len()) {
        let a = first.coefficients.get(i).map_or(&[][..], Vec::as_slice);
        let b = second.coefficients.get(i).map_or(&[][..], Vec::as_slice);
        for j in 0..a.len().max(b.len()) {
            let a = a.get(j).cloned().unwrap_or_else(Real::zero);
            let b = b.get(j).cloned().unwrap_or_else(Real::zero);
            assert!((a - b).definitely_zero());
        }
    }
}

fn verify_factors(topology: &BivariateCurveTopology, expected: &[(&BivariatePolynomial, usize)]) {
    let evidence = topology.repeated_factorization().unwrap();
    assert_eq!(evidence.factors().len(), expected.len());
    let mut rebuilt = BivariatePolynomial::new(vec![vec![evidence.scale().clone()]]);
    let mut support = polynomial(&[&[1]]);
    for (factor, &(known, exponent)) in evidence.factors().iter().zip(expected) {
        assert_eq!(factor.exponent(), exponent);
        // Normalize the independent expected factor in the same lexicographic
        // order, without invoking the factor-discovery or gcd implementation.
        let leading = known
            .coefficients
            .iter()
            .rev()
            .find_map(|r| r.iter().rev().find(|c| !c.definitely_zero()))
            .unwrap();
        let normalized = BivariatePolynomial::new(
            known
                .coefficients
                .iter()
                .map(|row| row.iter().map(|c| (c / leading).unwrap()).collect())
                .collect(),
        );
        assert_polynomial_eq(factor.equation(), &normalized);
        support = bivariate_multiply(&support, factor.equation());
        rebuilt = bivariate_multiply(&rebuilt, &power(factor.equation(), exponent));
    }
    assert_polynomial_eq(&rebuilt, topology.equation());
    assert_polynomial_eq(&support, topology.geometric_equation());
    assert_eq!(topology.geometric_equation(), evidence.support());
    for slab in topology.slabs() {
        for (arc, root) in slab.arcs().iter().zip(slab.roots()) {
            let selected = arc.repeated_factor_index().unwrap();
            for (index, &(known, _)) in expected.iter().enumerate() {
                let row = bivariate_specialize_first(known, slab.sample());
                let belongs = if let Some(exact) = &root.exact_root {
                    Real::eval_poly(&row, exact).definitely_zero()
                } else {
                    let sturm =
                        UnivariateSturmSequence::new(&row, PredicatePolicy::STRICT).unwrap();
                    let count = sturm
                        .count_distinct_roots(&root.lower, &root.upper, PredicatePolicy::STRICT)
                        .unwrap()
                        + usize::from(Real::eval_poly(&row, &root.lower).definitely_zero());
                    assert!(count <= 1);
                    count == 1
                };
                assert_eq!(belongs, index == selected);
            }
        }
    }
    for event in topology.events() {
        let memberships = event.repeated_factor_indices().unwrap();
        assert_eq!(memberships.len(), event.roots().len());
        assert!(memberships.iter().all(|indices| !indices.is_empty()));
    }
}

fn check_powers(support: BivariatePolynomial) {
    let control = build(&support);
    verify_fibers(&control, &support);
    assert!(control.repeated_factorization().is_none());
    for exponent in [2, 3] {
        for scale in [1, -3] {
            let mut original = power(&support, exponent);
            for coefficient in original.coefficients.iter_mut().flatten() {
                *coefficient *= Real::from(scale);
            }
            let topology = build(&original);
            assert_eq!(topology.equation(), &original);
            verify_factors(&topology, &[(&support, exponent)]);
            assert_eq!(topology.domain(), &rectangle());
            assert_eq!(layout(&topology), layout(&control));
            verify_fibers(&topology, &support);
        }
    }
}

#[test]
fn repeated_circle_keeps_both_selected_branches() {
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let control = build(&circle);
    assert_eq!(layout(&control).0, vec![0, 1, 1, 0]);
    assert_eq!(
        control
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![0, 2, 0]
    );
    check_powers(circle);
}

#[test]
fn repeated_parabola_keeps_its_two_sided_critical_fiber() {
    check_powers(polynomial(&[&[0, 0, 1], &[-1]]));
}

#[test]
fn repeated_node_keeps_all_four_incident_half_branches() {
    let node = polynomial(&[&[0, 0, 1], &[], &[-1]]);
    let control = build(&node);
    assert_eq!(layout(&control).0, vec![2, 1, 2]);
    assert_eq!(
        control
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![2, 2]
    );
    check_powers(node);
}

#[test]
fn repeated_cusp_keeps_its_selected_one_sided_branches() {
    check_powers(polynomial(&[&[0, 0, 1], &[], &[], &[-1]]));
}

#[test]
fn repeated_isolated_point_is_not_an_empty_curve() {
    let isolated = polynomial(&[&[0, 0, 1], &[], &[1]]);
    let control = build(&isolated);
    assert!(!control.is_empty());
    assert_eq!(layout(&control).0, vec![0, 1, 0]);
    assert!(control.slabs().iter().all(|slab| slab.arcs().is_empty()));
    check_powers(isolated);
}

#[test]
fn different_factor_multiplicities_keep_the_original_crossings() {
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let line = polynomial(&[&[0, 1]]);
    let support = bivariate_multiply(&circle, &line);
    let control = build(&support);
    assert_eq!(layout(&control).0, vec![1, 1, 1, 1]);
    assert_eq!(
        control
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![1, 3, 1]
    );
    let original = bivariate_multiply(&power(&circle, 2), &power(&line, 3));
    let topology = build(&original);
    assert_eq!(topology.equation(), &original);
    assert_eq!(layout(&topology), layout(&control));
    verify_fibers(&topology, &support);
    verify_factors(&topology, &[(&circle, 2), (&line, 3)]);
    assert_eq!(
        topology
            .events()
            .iter()
            .map(|event| event.repeated_factor_indices().unwrap().to_vec())
            .collect::<Vec<_>>(),
        vec![
            vec![vec![1]],
            vec![vec![0, 1]],
            vec![vec![0, 1]],
            vec![vec![1]]
        ]
    );
}

#[test]
fn rootless_axis_factors_keep_their_defining_powers() {
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    for axis in [polynomial(&[&[1], &[], &[1]]), polynomial(&[&[-3], &[1]])] {
        let original = bivariate_multiply(&power(&axis, 2), &power(&circle, 3));
        let support = bivariate_multiply(&axis, &circle);
        let topology = build(&original);
        assert_eq!(topology.equation(), &original);
        verify_factors(&topology, &[(&axis, 2), (&circle, 3)]);
        assert_eq!(layout(&topology), layout(&build(&support)));
        verify_fibers(&topology, &support);
    }
}

#[test]
fn repeated_components_preserve_degree_budget_and_retry() {
    use hypersolve::bivariate_topology::BivariateTopologyError;
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let original = power(&polynomial(&[&[-1, 0, 1], &[], &[1]]), 3);
    for limits in [
        BivariateTopologyConfig {
            max_axis_degree: 5,
            ..config()
        },
        BivariateTopologyConfig {
            max_projection_degree: 2,
            ..config()
        },
    ] {
        assert_eq!(
            decompose_bivariate_curve(&original, &rectangle(), limits),
            Err(BivariateTopologyError::DegreeLimit)
        );
    }
    let denied = with_budget(WorkBudget::ZERO, || {
        assert_eq!(
            decompose_bivariate_curve(&original, &rectangle(), config()),
            Err(BivariateTopologyError::BudgetExhausted)
        );
    });
    assert!(denied.result.is_err());
    let during_reduction = with_budget(
        WorkBudget {
            promotions: 3,
            refinements: 1_000_000,
        },
        || {
            assert_eq!(
                decompose_bivariate_curve(&original, &rectangle(), config()),
                Err(BivariateTopologyError::BudgetExhausted)
            );
        },
    );
    assert!(during_reduction.result.is_err());
    let retry = with_budget(
        WorkBudget {
            promotions: 100_000,
            refinements: 1_000_000,
        },
        || build(&original),
    );
    let restored = retry.result.unwrap();
    assert_eq!(restored.equation(), &original);
    assert_eq!(layout(&restored), layout(&build(&original)));
    assert!(retry.usage.promotions > 1);
}
