//! Closed rectangle boundaries are part of the original zero-set domain.

use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::{bivariate_specialize_first, bivariate_specialize_second};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateCurveTopology, BivariateTopologyConfig, BivariateTopologyError,
    decompose_bivariate_curve,
};
use hypersolve::{BivariatePolynomial, RealInterval, UnivariateSturmSequence, work_budget};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn rectangle(x0: i32, x1: i32, y0: i32, y1: i32) -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [x0.into(), x1.into()],
        second: [y0.into(), y1.into()],
    }
}

fn closed_count(row: &[Real], lower: &Real, upper: &Real) -> usize {
    // The rational Sturm API counts (lower, upper]; closed fibers include
    // the lower root separately. This does not use the algebraic-fiber counter.
    UnivariateSturmSequence::new(row, STRICT)
        .unwrap()
        .count_distinct_roots(lower, upper, STRICT)
        .unwrap()
        + usize::from(Real::eval_poly(row, lower).definitely_zero())
}

fn audit(topology: &BivariateCurveTopology) {
    let domain = topology.domain();
    for slab in topology.slabs() {
        let row = bivariate_specialize_first(topology.equation(), slab.sample());
        assert_eq!(
            closed_count(&row, &domain.second[0], &domain.second[1]),
            slab.arcs().len()
        );
        assert_eq!(slab.roots().len(), slab.arcs().len());
        for proof in slab.connections().iter().flatten() {
            for y in proof.separators() {
                let row = bivariate_specialize_second(topology.equation(), y);
                assert!(
                    RealInterval::evaluate_power_basis(&row, proof.first_interval())
                        .unwrap()
                        .strict_nonzero_sign()
                        .is_some()
                );
            }
            let row = bivariate_specialize_first(topology.equation(), proof.sample());
            for (band, &count) in proof.separators().windows(2).zip(proof.root_counts()) {
                let lower = if band[0].exact_rational_normal_form().unwrap()
                    < domain.second[0].exact_rational_normal_form().unwrap()
                {
                    &domain.second[0]
                } else {
                    &band[0]
                };
                let upper = if band[1].exact_rational_normal_form().unwrap()
                    > domain.second[1].exact_rational_normal_form().unwrap()
                {
                    &domain.second[1]
                } else {
                    &band[1]
                };
                assert_eq!(closed_count(&row, lower, upper), count);
            }
            assert_eq!(proof.root_counts().iter().sum::<usize>(), slab.arcs().len());
        }
    }
    for event in topology.events() {
        if let Some(x) = event.parameter().exact_point_witness() {
            let row = bivariate_specialize_first(topology.equation(), x);
            assert_eq!(
                closed_count(&row, &domain.second[0], &domain.second[1]),
                event.roots().len()
            );
        }
    }
}

fn build(equation: BivariatePolynomial, domain: BivariateCurveDomain) -> BivariateCurveTopology {
    let topology =
        decompose_bivariate_curve(&equation, &domain, BivariateTopologyConfig::default()).unwrap();
    assert_eq!(topology.equation(), &equation);
    assert_eq!(topology.domain(), &domain);
    audit(&topology);
    assert_eq!(
        decompose_bivariate_curve(&equation, &domain, topology.config()).unwrap(),
        topology
    );
    topology
}

fn incidence(topology: &BivariateCurveTopology) -> Vec<Vec<(usize, usize)>> {
    let mut result: Vec<Vec<_>> = topology
        .events()
        .iter()
        .map(|event| vec![(0, 0); event.roots().len()])
        .collect();
    for (index, slab) in topology.slabs().iter().enumerate() {
        for arc in slab.arcs() {
            result[index][arc.left_root()].1 += 1;
            result[index + 1][arc.right_root()].0 += 1;
        }
    }
    result
}

fn domain() -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-2).into(), 2.into()],
    }
}

fn assert_horizontal_topology(topology: &BivariateCurveTopology, ordinates: &[i32]) {
    audit(topology);
    assert_eq!(topology.domain(), &domain());
    assert_eq!(topology.events().len(), 2);
    assert_eq!(topology.slabs().len(), 1);
    for event in topology.events() {
        assert_eq!(event.roots().len(), ordinates.len());
        for (root, y) in event.roots().iter().zip(ordinates) {
            assert_eq!(root.exact_root.as_ref(), Some(&Real::from(*y)));
        }
    }
    let slab = &topology.slabs()[0];
    assert_eq!(slab.arcs().len(), ordinates.len());
    for (index, arc) in slab.arcs().iter().enumerate() {
        assert_eq!((arc.left_root(), arc.right_root()), (index, index));
    }
    assert!(slab.connections().iter().all(Option::is_some));
    for proof in slab.connections().iter().flatten() {
        assert_eq!(proof.root_counts(), vec![1; ordinates.len()]);
    }
    assert_eq!(
        topology
            .rational_fiber(&Real::zero(), Default::default())
            .unwrap()
            .roots()
            .len(),
        ordinates.len()
    );
    assert_eq!(
        decompose_bivariate_curve(topology.equation(), topology.domain(), topology.config())
            .unwrap(),
        *topology
    );
}

#[test]
fn original_horizontal_boundary_component_is_a_complete_closed_graph() {
    // Preserve the original lower-boundary blocker fixture and its reflected upper boundary.
    for (constant, ordinate) in [(2, -2), (-2, 2)] {
        let equation = BivariatePolynomial::new(vec![vec![constant.into(), 1.into()]]);
        let topology =
            decompose_bivariate_curve(&equation, &domain(), BivariateTopologyConfig::default())
                .unwrap();
        assert_eq!(topology.equation(), &equation);
        assert_horizontal_topology(&topology, &[ordinate]);
    }
}

#[test]
fn both_horizontal_boundary_factors_retain_distinct_components() {
    let equation = BivariatePolynomial::new(vec![vec![(-4).into(), 0.into(), 1.into()]]);
    let topology =
        decompose_bivariate_curve(&equation, &domain(), BivariateTopologyConfig::default())
            .unwrap();
    assert_eq!(topology.equation(), &equation);
    assert_horizontal_topology(&topology, &[-2, 2]);
}

#[test]
fn boundary_tangency_retains_both_incident_graphs() {
    // y(y-x^2)=0: both branches are inside, with a four-valent contact at (0,0).
    let topology = build(
        polynomial(&[&[0, 0, 1], &[], &[0, -1]]),
        rectangle(-1, 1, 0, 2),
    );
    assert_eq!(
        incidence(&topology),
        vec![vec![(0, 1), (0, 1)], vec![(2, 2)], vec![(1, 0), (1, 0)]]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![2, 2]
    );
    for slab in topology.slabs() {
        let fiber = topology
            .rational_fiber(slab.sample(), Default::default())
            .unwrap();
        assert_eq!(fiber.roots()[0].exact_root.as_ref(), Some(&Real::zero()));
        assert_eq!(
            fiber.roots()[1].exact_root.as_ref(),
            Some(&(slab.sample() * slab.sample()))
        );
    }
}

#[test]
fn boundary_crossing_retains_only_the_closed_domain_sides() {
    // y(y-x)=0: the slanted branch enters at the boundary node and exits at (1,1).
    let topology = build(polynomial(&[&[0, 0, 1], &[0, -1]]), rectangle(-1, 1, 0, 1));
    assert_eq!(
        incidence(&topology),
        vec![vec![(0, 1)], vec![(1, 2)], vec![(1, 0), (1, 0)]]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![1, 2]
    );
}

#[test]
fn circle_tangent_to_boundary_component_keeps_loop_and_line() {
    // y * (x^2 + (y-1)^2 - 1): the circle touches the retained y=0 line.
    let topology = build(
        polynomial(&[&[0, 0, -2, 1], &[], &[0, 1]]),
        rectangle(-2, 2, 0, 3),
    );
    assert_eq!(
        incidence(&topology),
        vec![
            vec![(0, 1)],
            vec![(1, 1), (0, 2)],
            vec![(2, 2), (1, 1)],
            vec![(1, 1), (2, 0)],
            vec![(1, 0)],
        ]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![1, 3, 3, 1]
    );
    for (event, x) in topology.events().iter().zip([-2, -1, 0, 1, 2]) {
        assert_eq!(
            event.parameter().exact_point_witness(),
            Some(&Real::from(x))
        );
    }
}

#[test]
fn original_circle_and_upper_boundary_fixture_retains_both_components() {
    // Original documented blocker: (y-1)(16x^2+16y^2-4) in [-1,1]^2.
    let topology = build(
        polynomial(&[&[4, -4, -16, 16], &[], &[-16, 16]]),
        rectangle(-1, 1, -1, 1),
    );
    assert_eq!(
        incidence(&topology),
        vec![
            vec![(0, 1)],
            vec![(0, 2), (1, 1)],
            vec![(2, 0), (1, 1)],
            vec![(1, 0)]
        ]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![1, 3, 1]
    );
}

#[test]
fn algebraic_boundary_crossings_keep_selected_event_fibers() {
    // y(y-x^2+2)=0 enters the upper half-plane at x=+/-sqrt(2).
    let topology = build(
        polynomial(&[&[0, 2, 1], &[], &[0, -1]]),
        rectangle(-2, 2, 0, 3),
    );
    assert_eq!(
        incidence(&topology),
        vec![
            vec![(0, 1), (0, 1)],
            vec![(2, 1)],
            vec![(1, 2)],
            vec![(1, 0), (1, 0)],
        ]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![2, 1, 2]
    );
    for event in &topology.events()[1..3] {
        assert!(event.parameter().exact_point_witness().is_none());
        assert!(event.parameter().is_valid());
        assert_eq!(event.roots().len(), 1);
        assert_eq!(event.roots()[0].exact_root.as_ref(), Some(&Real::zero()));
        // The only interior roots of the projection satisfy x^2=2.
        assert_eq!(
            closed_count(
                &[(-2).into(), 0.into(), 1.into()],
                &event.parameter().interval.lower,
                &event.parameter().interval.upper
            ),
            1
        );
    }
}

#[test]
fn nearby_boundary_and_interior_components_do_not_merge() {
    let small = (Real::one() / Real::from(1 << 20)).unwrap();
    let small = &small * &small;
    let small = &small * &small; // 2^-80, with a third component just outside.
    let equation =
        BivariatePolynomial::new(vec![vec![0.into(), -(&small * &small), 0.into(), 1.into()]]);
    let topology = build(equation, rectangle(-1, 1, 0, 1));
    assert_eq!(
        incidence(&topology),
        vec![vec![(0, 1), (0, 1)], vec![(1, 0), (1, 0)]]
    );
    let fiber = topology
        .rational_fiber(&Real::zero(), Default::default())
        .unwrap();
    assert_eq!(fiber.roots().len(), 2);
    assert_eq!(fiber.roots()[0].exact_root.as_ref(), Some(&Real::zero()));
    assert_eq!(fiber.roots()[1].exact_root.as_ref(), Some(&small));
}

#[test]
fn boundary_components_preserve_budget_failure_and_retry() {
    let equation = polynomial(&[&[0, 0, -2, 1], &[], &[0, 1]]);
    let rectangle = rectangle(-2, 2, 0, 3);
    let denied = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        let result = decompose_bivariate_curve(&equation, &rectangle, Default::default());
        assert_eq!(result, Err(BivariateTopologyError::BudgetExhausted));
        result
    });
    assert_eq!(
        denied.result.unwrap_err(),
        work_budget::BudgetExceeded {
            kind: work_budget::WorkKind::AlgebraicPromotion,
            limit: 0,
            used: 0,
            requested: 1
        }
    );
    assert_eq!(denied.usage, work_budget::WorkUsage::default());
    let recovered = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 100000,
            promotions: 100000,
        },
        || decompose_bivariate_curve(&equation, &rectangle, Default::default()),
    );
    let topology = recovered.result.unwrap().unwrap();
    assert_eq!(topology, build(equation, rectangle));
}

#[test]
fn boundary_support_does_not_admit_repeated_or_vertical_components() {
    let rectangle = rectangle(-1, 1, 0, 1);
    for (equation, error) in [
        (
            polynomial(&[&[0, 0, 1]]),
            BivariateTopologyError::RepeatedComponent,
        ),
        (
            polynomial(&[&[], &[0, 1]]),
            BivariateTopologyError::VerticalComponent,
        ),
        (polynomial(&[&[0]]), BivariateTopologyError::ZeroPolynomial),
    ] {
        assert_eq!(
            decompose_bivariate_curve(&equation, &rectangle, Default::default()),
            Err(error)
        );
    }
}
