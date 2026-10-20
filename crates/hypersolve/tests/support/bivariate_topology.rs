use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::bivariate_topology::*;
use hypersolve::{AlgebraicRootRepresentation, BivariatePolynomial, work_budget};
use std::cmp::Ordering;
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

use serde_json::{Value, json};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;
use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_swap_parameters};

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn polynomial(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn domain(x0: i64, x1: i64, y0: i64, y1: i64) -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [x0.into(), x1.into()],
        second: [y0.into(), y1.into()],
    }
}

fn build(equation: &BivariatePolynomial, domain: &BivariateCurveDomain) -> BivariateCurveTopology {
    let topology =
        decompose_bivariate_curve(equation, domain, BivariateTopologyConfig::default()).unwrap();
    audit_and_materialize(&topology);
    topology
}

fn rational(value: &Real) -> String {
    value.exact_rational_normal_form().unwrap().to_string()
}

fn coefficients(values: &[Real]) -> Vec<String> {
    values.iter().map(rational).collect()
}

fn interval(root: &hypersolve::IsolatedRootInterval) -> Value {
    json!({"lower":rational(&root.lower),"upper":rational(&root.upper),
        "exact":root.exact_root.as_ref().map(rational),"count":root.distinct_root_count})
}

fn audit_and_materialize(topology: &BivariateCurveTopology) {
    use hypersolve::bivariate_arithmetic::{
        bivariate_specialize_first, bivariate_specialize_second,
    };
    use hypersolve::{
        AlgebraicFiberRootCountStatus, CurveResultantParameter, RealInterval,
        UnivariateSturmSequence,
    };

    let domain = topology.domain();
    let projection = UnivariateSturmSequence::new(topology.projection(), STRICT).unwrap();
    assert_eq!(
        projection.count_distinct_roots(&domain.first[0], &domain.first[1], STRICT),
        Some(topology.events().len() - 1)
    );
    for event in topology.events() {
        let root = event.parameter();
        if let Some(value) = root.exact_point_witness() {
            assert!(Real::eval_poly(topology.projection(), value).definitely_zero());
        } else {
            assert_eq!(
                projection.count_distinct_roots(&root.interval.lower, &root.interval.upper, STRICT),
                Some(1)
            );
        }
        let count = hypersolve::count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            topology.equation(),
            CurveResultantParameter::First,
            root,
            &domain.second[0],
            &domain.second[1],
            STRICT,
        );
        assert_eq!(count.status, AlgebraicFiberRootCountStatus::Counted);
        assert_eq!(count.distinct_root_count, Some(event.roots().len()));
    }
    for slab in topology.slabs() {
        for proof in slab.connections() {
            let Some(proof) = proof else {
                assert!(slab.arcs().is_empty());
                continue;
            };
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
            let sequence = UnivariateSturmSequence::new(&row, STRICT).unwrap();
            let cmp = |a: &Real, b: &Real| {
                a.exact_rational_normal_form()
                    .unwrap()
                    .partial_cmp(&b.exact_rational_normal_form().unwrap())
                    .unwrap()
            };
            for (pair, &expected) in proof.separators().windows(2).zip(proof.root_counts()) {
                let lower = if cmp(&pair[0], &domain.second[0]) == Ordering::Less {
                    &domain.second[0]
                } else {
                    &pair[0]
                };
                let upper = if cmp(&pair[1], &domain.second[1]) == Ordering::Greater {
                    &domain.second[1]
                } else {
                    &pair[1]
                };
                assert!(!Real::eval_poly(&row, lower).definitely_zero());
                assert!(!Real::eval_poly(&row, upper).definitely_zero());
                assert_eq!(
                    sequence.count_distinct_roots(lower, upper, STRICT),
                    Some(expected)
                );
            }
            assert_eq!(proof.root_counts().iter().sum::<usize>(), slab.arcs().len());
        }
    }
    // This diagnostic encoding retains exact rational payloads and all graph
    // evidence. It is not the BRep native archive or a trusted deserialization API.
    let record = json!({
        "equation":topology.equation().coefficients.iter().map(|r|coefficients(r)).collect::<Vec<_>>(),
        "domain":{"first":coefficients(&domain.first),"second":coefficients(&domain.second)},
        "projection":coefficients(topology.projection()),
        "events":topology.events().iter().map(|event|json!({
            "parameter":interval(&event.parameter().interval),
            "parameter_equation":coefficients(&event.parameter().polynomial_coefficients),
            "ordinal":event.parameter().interval_index,
            "roots":event.roots().iter().map(interval).collect::<Vec<_>>()
        })).collect::<Vec<_>>(),
        "slabs":topology.slabs().iter().map(|slab|json!({
            "sample":rational(slab.sample()),
            "roots":slab.roots().iter().map(interval).collect::<Vec<_>>(),
            "arcs":slab.arcs().iter().map(|a|[a.left_root(),a.right_root()]).collect::<Vec<_>>(),
            "connections":slab.connections().iter().map(|proof|proof.as_ref().map(|proof|json!({
                "interval":[rational(&proof.first_interval().lower),rational(&proof.first_interval().upper)],
                "sample":rational(proof.sample()),"separators":coefficients(proof.separators()),"counts":proof.root_counts()
            }))).collect::<Vec<_>>()
        })).collect::<Vec<_>>()
    });
    let bytes = serde_json::to_vec(&record).unwrap();
    if let Some(directory) = std::env::var_os("HYPERSOLVE_BIVARIATE_TOPOLOGY_ARTIFACT_DIR") {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let index = NEXT.fetch_add(1, AtomicOrdering::Relaxed);
        std::fs::write(
            std::path::PathBuf::from(directory).join(format!("{index:04}.json")),
            &bytes,
        )
        .unwrap();
    }
    std::hint::black_box(bytes);
}

fn incidence(topology: &BivariateCurveTopology) -> Vec<Vec<(usize, usize)>> {
    let mut result: Vec<Vec<_>> = topology
        .events()
        .iter()
        .map(|event| vec![(0, 0); event.roots().len()])
        .collect();
    for (index, slab) in topology.slabs().iter().enumerate() {
        assert_eq!(slab.arcs().len(), slab.roots().len());
        for arc in slab.arcs() {
            result[index][arc.left_root()].1 += 1;
            result[index + 1][arc.right_root()].0 += 1;
        }
    }
    result
}

pub fn complete_interior_loop_and_empty_fibers() {
    // x^2+y^2=1 has no boundary contact in [-2,2]^2. Two critical
    // points and two ordered branches close a loop without a boundary seed.
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let rectangle = domain(-2, 2, -2, 2);
    let topology = build(&circle, &rectangle);
    assert_eq!(topology.events().len(), 4);
    assert_eq!(
        incidence(&topology),
        vec![vec![], vec![(0, 2)], vec![(2, 0)], vec![]]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|slab| slab.arcs().len())
            .collect::<Vec<_>>(),
        vec![0, 2, 0]
    );
    for (event, x) in topology.events().iter().zip([-2, -1, 1, 2]) {
        let comparison = hypersolve::compare_algebraic_root_representations_by_difference(
            event.parameter(),
            &AlgebraicRootRepresentation::from_exact_value(&Real::from(x)),
            hypersolve::AlgebraicRootRefinementComparisonConfig {
                policy: STRICT,
                ..Default::default()
            },
        );
        assert_eq!(comparison.comparison.ordering, Some(Ordering::Equal));
    }
    assert!(!topology.is_empty());
    // Replaying the source/domain, rather than accepting stored adjacency,
    // reconstructs all exact fields and deterministic connectivity.
    assert_eq!(build(topology.equation(), topology.domain()), topology);
    assert!(build(&circle, &domain(2, 3, -1, 1)).is_empty());
    assert!(build(&polynomial(&[&[1, 0, 1], &[], &[1]]), &rectangle).is_empty());
    assert!(build(&polynomial(&[&[2]]), &rectangle).is_empty());
}

pub fn algebraic_fiber_events_and_quartic_projection_loops() {
    // x^2+(y^2-2)^2=1: two disjoint ovals whose covertical event
    // points have nonrational y coordinates +/-sqrt(2).
    let topology = build(
        &polynomial(&[&[3, 0, -4, 0, 1], &[], &[1]]),
        &domain(-2, 2, -2, 2),
    );
    assert_eq!(
        incidence(&topology),
        vec![vec![], vec![(0, 2), (0, 2)], vec![(2, 0), (2, 0)], vec![]]
    );
    assert!(
        topology.events()[1]
            .roots()
            .iter()
            .all(|root| root.exact_root.is_none())
    );
    // y^2+x^4-x^2+1/8=0: four algebraic projection extrema and
    // two interior components. The quartic has exactly four simple real roots.
    let quartic = BivariatePolynomial::new(vec![
        vec![q(1, 8), 0.into(), 1.into()],
        vec![],
        vec![(-1).into()],
        vec![],
        vec![1.into()],
    ]);
    let topology = build(&quartic, &domain(-2, 2, -2, 2));
    assert_eq!(
        incidence(&topology),
        vec![
            vec![],
            vec![(0, 2)],
            vec![(2, 0)],
            vec![(0, 2)],
            vec![(2, 0)],
            vec![]
        ]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![0, 2, 0, 2, 0]
    );
}

pub fn irreducible_nodal_quartic_keeps_both_loops() {
    // (x^2+y^2)^2-2(x^2-y^2)=0 is a figure-eight with one node
    // and two loops. Keeping only boundary contacts would lose the whole set.
    let source = polynomial(&[&[0, 0, 2, 0, 1], &[], &[-2, 0, 2], &[], &[1]]);
    let topology = build(&source, &domain(-2, 2, -2, 2));
    assert_eq!(
        incidence(&topology),
        vec![vec![], vec![(0, 2)], vec![(2, 2)], vec![(2, 0)], vec![]]
    );
    assert_eq!(
        topology
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![0, 2, 2, 0]
    );
    assert_eq!(build(topology.equation(), topology.domain()), topology);
}

pub fn covertical_critical_points_and_disconnected_loops() {
    // Two unit circles centered at (0,+/-2). Both left extrema and
    // both right extrema share a projection value: no genericity assumption.
    let first = polynomial(&[&[3, -4, 1], &[], &[1]]);
    let second = polynomial(&[&[3, 4, 1], &[], &[1]]);
    let source = bivariate_multiply(&first, &second);
    let topology = build(&source, &domain(-2, 2, -4, 4));
    assert_eq!(
        incidence(&topology),
        vec![vec![], vec![(0, 2), (0, 2)], vec![(2, 0), (2, 0)], vec![]]
    );
    let arcs = topology.slabs()[1].arcs();
    assert_eq!(
        arcs.iter()
            .map(|a| (a.left_root(), a.right_root()))
            .collect::<Vec<_>>(),
        vec![(0, 0), (0, 0), (1, 1), (1, 1)]
    );
    assert_eq!(build(&source, &domain(-2, 2, -4, 4)), topology);
}

pub fn singular_nodes_cusps_and_isolated_contacts_keep_incidence() {
    let rectangle = domain(-1, 1, -2, 2);
    let node = build(&polynomial(&[&[0, 0, 1], &[], &[-1]]), &rectangle);
    assert_eq!(
        incidence(&node),
        vec![vec![(0, 1), (0, 1)], vec![(2, 2)], vec![(1, 0), (1, 0)]]
    );
    let cusp = build(&polynomial(&[&[0, 0, 1], &[], &[], &[-1]]), &rectangle);
    assert_eq!(
        incidence(&cusp),
        vec![vec![], vec![(0, 2)], vec![(1, 0), (1, 0)]]
    );
    let isolated = build(&polynomial(&[&[0, 0, 1], &[], &[1]]), &rectangle);
    assert_eq!(incidence(&isolated), vec![vec![], vec![(0, 0)], vec![]]);
    assert!(!isolated.is_empty());
}

pub fn trim_boundary_crossings_tangencies_and_corners() {
    // y=x crosses two corners; boundary roots are owned exactly once.
    let line = build(&polynomial(&[&[0, 1], &[-1]]), &domain(-1, 1, -1, 1));
    assert_eq!(incidence(&line), vec![vec![(0, 1)], vec![(1, 0)]]);
    // The lower semicircle meets the horizontal boundary at both extrema.
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let half = build(&circle, &domain(-2, 2, -2, 0));
    assert_eq!(
        incidence(&half),
        vec![vec![], vec![(0, 1)], vec![(1, 0)], vec![]]
    );
    // The upper half-plane touches only the top of the circle.
    let touch = build(&circle, &domain(-2, 2, 1, 2));
    assert_eq!(
        incidence(&touch).into_iter().flatten().collect::<Vec<_>>(),
        vec![(0, 0)]
    );
    // y=x^2 touches the lower boundary while both branches remain inside.
    let parabola = build(&polynomial(&[&[0, 1], &[], &[-1]]), &domain(-1, 1, 0, 2));
    assert_eq!(
        incidence(&parabola),
        vec![vec![(0, 1)], vec![(1, 1)], vec![(1, 0)]]
    );
}

pub fn degree_drop_and_algebraic_projection_events() {
    // xy=1 has a degree drop at x=0 and two clipped, disconnected branches.
    let hyperbola = build(&polynomial(&[&[-1], &[0, 1]]), &domain(-2, 2, -2, 2));
    assert_eq!(
        hyperbola
            .slabs()
            .iter()
            .map(|s| s.arcs().len())
            .collect::<Vec<_>>(),
        vec![1, 0, 0, 1]
    );
    assert_eq!(
        incidence(&hyperbola),
        vec![
            vec![(0, 1)],
            vec![(1, 0)],
            vec![],
            vec![(0, 1)],
            vec![(1, 0)]
        ]
    );
    // Extrema at x=+/-sqrt(2) are retained roots, not scalar approximations.
    let circle = polynomial(&[&[-2, 0, 1], &[], &[1]]);
    let topology = build(&circle, &domain(-2, 2, -2, 2));
    assert_eq!(
        incidence(&topology),
        vec![vec![], vec![(0, 2)], vec![(2, 0)], vec![]]
    );
    assert!(
        topology.events()[1]
            .parameter()
            .exact_point_witness()
            .is_none()
    );
    assert!(
        topology.events()[2]
            .parameter()
            .exact_point_witness()
            .is_none()
    );
    assert_eq!(build(&circle, &domain(-2, 2, -2, 2)), topology);
}

pub fn original_brep_patch_pullback_and_chart_changes() {
    // Exact pullback of the unchanged BRep interior-loop repro.
    let source = BivariatePolynomial::new(vec![
        vec![q(7, 16), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let unit = domain(0, 1, 0, 1);
    let original = build(&source, &unit);
    assert_eq!(
        incidence(&original),
        vec![vec![], vec![(0, 2)], vec![(2, 0)], vec![]]
    );
    for source in [
        bivariate_swap_parameters(&source),
        source.substitute_affine(&(-1).into(), &1.into(), &1.into(), &0.into()),
    ] {
        assert_eq!(incidence(&build(&source, &unit)), incidence(&original));
    }
    // A tiny loop is kept at the original scale. No tolerance or sampling
    // density changes its component count or closes the gap to the boundary.
    let tiny = BivariatePolynomial::new(vec![
        vec![q(1, 2) - q(1, 1_000_000_000_000), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let topology = build(&tiny, &unit);
    assert_eq!(incidence(&topology), incidence(&original));
}

pub fn explicit_dimension_degeneracy_budget_and_recovery() {
    let rectangle = domain(-2, 2, -2, 2);
    let config = BivariateTopologyConfig::default();
    for (source, expected) in [
        (polynomial(&[&[0]]), BivariateTopologyError::ZeroPolynomial),
        (
            polynomial(&[&[0], &[1]]),
            BivariateTopologyError::VerticalComponent,
        ),
        (
            polynomial(&[&[0, 0, 1]]),
            BivariateTopologyError::RepeatedComponent,
        ),
        (
            polynomial(&[&[2, 1]]),
            BivariateTopologyError::BoundaryComponent,
        ),
    ] {
        assert_eq!(
            decompose_bivariate_curve(&source, &rectangle, config),
            Err(expected)
        );
    }
    let nonrational = BivariatePolynomial::new(vec![vec![Real::pi(), 1.into()]]);
    assert_eq!(
        decompose_bivariate_curve(&nonrational, &rectangle, config),
        Err(BivariateTopologyError::UnsupportedCoefficient)
    );
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    assert_eq!(
        decompose_bivariate_curve(&circle, &domain(1, 1, -1, 1), config),
        Err(BivariateTopologyError::InvalidDomain)
    );
    assert_eq!(
        decompose_bivariate_curve(
            &circle,
            &rectangle,
            BivariateTopologyConfig {
                max_axis_degree: 1,
                ..config
            }
        ),
        Err(BivariateTopologyError::DegreeLimit)
    );
    assert_eq!(
        decompose_bivariate_curve(
            &circle,
            &rectangle,
            BivariateTopologyConfig {
                max_projection_degree: 1,
                ..config
            }
        ),
        Err(BivariateTopologyError::DegreeLimit)
    );
    let denied = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        decompose_bivariate_curve(&circle, &rectangle, config)
    });
    assert!(denied.result.is_err());
    assert_eq!(denied.usage, work_budget::WorkUsage::default());
    let recovered = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 10000,
            promotions: 10000,
        },
        || decompose_bivariate_curve(&circle, &rectangle, config),
    );
    assert!(!recovered.result.unwrap().unwrap().is_empty());
    assert_eq!(build(&circle, &rectangle), build(&circle, &rectangle));
}

pub fn source_bound_slices_limits_and_recovery() {
    let source = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let rectangle = domain(-2, 2, -2, 2);
    let config = BivariateTopologyConfig::default();
    let topology = build(&source, &rectangle);
    for (x, count) in [
        (q(-3, 2), 0),
        (q(-1, 1), 1),
        (q(-1, 2), 2),
        (q(0, 1), 2),
        (q(1, 2), 2),
        (q(1, 1), 1),
        (q(3, 2), 0),
    ] {
        let fiber = topology.rational_fiber(&x, config.fiber).unwrap();
        assert_eq!(fiber.parameter().exact_point_witness(), Some(&x));
        assert_eq!(fiber.roots().len(), count);
        assert_eq!(fiber, topology.rational_fiber(&x, config.fiber).unwrap());
        // An independently constructed univariate Sturm chain checks the
        // completeness of every returned rational slice and each singleton.
        let row = [(&x * &x) - Real::one(), Real::zero(), Real::one()];
        let sturm = hypersolve::UnivariateSturmSequence::new(&row, STRICT).unwrap();
        assert_eq!(
            sturm.count_distinct_roots(&(-2).into(), &2.into(), STRICT),
            Some(count)
        );
        for root in fiber.roots() {
            if let Some(value) = &root.exact_root {
                assert!(Real::eval_poly(&row, value).definitely_zero());
            } else {
                assert_eq!(
                    sturm.count_distinct_roots(&root.lower, &root.upper, STRICT),
                    Some(1)
                );
            }
        }
    }
    assert_eq!(
        topology.rational_fiber(&3.into(), config.fiber),
        Err(BivariateTopologyError::ParameterOutsideDomain)
    );
    assert_eq!(
        topology.rational_fiber(&Real::pi(), config.fiber),
        Err(BivariateTopologyError::UnsupportedCoefficient)
    );
    let denied = decompose_bivariate_curve(
        &source,
        &rectangle,
        BivariateTopologyConfig {
            fiber: hypersolve::AlgebraicFiberRootIsolationConfig {
                max_subdivision_depth: 0,
                refinement_steps: 0,
            },
            ..config
        },
    );
    assert_eq!(denied, Err(BivariateTopologyError::RefinementLimit));
    let budgeted = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        topology.rational_fiber(&q(1, 2), config.fiber)
    });
    assert!(budgeted.result.is_err());
    assert_eq!(
        topology
            .rational_fiber(&q(1, 2), config.fiber)
            .unwrap()
            .roots()
            .len(),
        2
    );
    assert_eq!(topology, build(&source, &rectangle));
}
