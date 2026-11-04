//! Complete vertical intervals and residual attachments, with exact source powers.
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_specialize_first};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateCurveTopology, BivariateTopologyConfig, BivariateTopologyError,
    decompose_bivariate_curve,
};
use hypersolve::{BivariatePolynomial, UnivariateSturmSequence};

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn axis(values: &[Real]) -> BivariatePolynomial {
    BivariatePolynomial::new(values.iter().map(|x| vec![x.clone()]).collect())
}
fn power(p: &BivariatePolynomial, n: usize) -> BivariatePolynomial {
    (0..n).fold(polynomial(&[&[1]]), |a, _| bivariate_multiply(&a, p))
}
fn rectangle() -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-3).into(), 3.into()],
    }
}
fn build(p: &BivariatePolynomial, domain: &BivariateCurveDomain) -> BivariateCurveTopology {
    let t = decompose_bivariate_curve(p, domain, Default::default()).unwrap();
    assert_eq!(t.equation(), p);
    assert_eq!(t.domain(), domain);
    let v = t
        .vertical_content()
        .expect("live vertical roots retain their content");
    equal_polynomials(
        p,
        &bivariate_multiply(&axis(v.factor()), v.residual_equation()),
    );
    // Independent univariate root census on the exact content, with explicit
    // lower-end ownership because the Sturm query is half open.
    let sturm = UnivariateSturmSequence::new(v.factor(), PredicatePolicy::STRICT).unwrap();
    let mut count = sturm
        .count_distinct_roots(&domain.first[0], &domain.first[1], PredicatePolicy::STRICT)
        .unwrap();
    let lower = bivariate_specialize_first(&axis(v.factor()), &domain.first[0]);
    if lower.iter().all(|x| {
        x.exact_rational_normal_form().unwrap()
            == Real::zero().exact_rational_normal_form().unwrap()
    }) {
        count += 1;
    }
    assert_eq!(t.vertical_components().len(), count);
    assert!(!t.is_empty());
    let replay = decompose_bivariate_curve(t.equation(), t.domain(), t.config()).unwrap();
    assert_eq!(
        t, replay,
        "all component selectors and residual incidences replay"
    );
    t
}
fn equal_polynomials(a: &BivariatePolynomial, b: &BivariatePolynomial) {
    let zero = Real::zero();
    for i in 0..a.coefficients.len().max(b.coefficients.len()) {
        let ar = a.coefficients.get(i).map_or(&[][..], Vec::as_slice);
        let br = b.coefficients.get(i).map_or(&[][..], Vec::as_slice);
        for j in 0..ar.len().max(br.len()) {
            assert_eq!(
                ar.get(j)
                    .unwrap_or(&zero)
                    .exact_rational_normal_form()
                    .unwrap(),
                br.get(j)
                    .unwrap_or(&zero)
                    .exact_rational_normal_form()
                    .unwrap()
            );
        }
    }
}
fn interval_contains(interval: &hypersolve::IsolatedRootInterval, value: &Real) {
    assert!(
        !hyperlimit::compare_reals(value, &interval.lower, PredicatePolicy::STRICT)
            .value()
            .unwrap()
            .is_lt()
    );
    assert!(
        !hyperlimit::compare_reals(value, &interval.upper, PredicatePolicy::STRICT)
            .value()
            .unwrap()
            .is_gt()
    );
}
fn parameter_is(root: &hypersolve::AlgebraicRootRepresentation, value: &Real) {
    assert!(root.is_valid());
    interval_contains(&root.interval, value);
    assert_eq!(
        UnivariateSturmSequence::new(&root.polynomial_coefficients, PredicatePolicy::STRICT)
            .unwrap()
            .classify_point(value, PredicatePolicy::STRICT),
        Some(hypersolve::UnivariateSturmPoint::Root)
    );
}
fn fiber_root_is(topology: &BivariateCurveTopology, event: usize, ordinal: usize, value: &Real) {
    let fiber = &topology.events()[event];
    interval_contains(&fiber.roots()[ordinal], value);
    let mut field = hypersolve::AlgebraicField::new(fiber.parameter()).unwrap();
    let coefficients = hypersolve::bivariate_arithmetic::bivariate_specialize_second(
        topology.graph_equation(),
        value,
    );
    let image = field
        .rational_function(fiber.parameter(), coefficients, vec![Real::one()])
        .unwrap();
    assert_eq!(field.sign(&image).unwrap(), std::cmp::Ordering::Equal);
    if fiber.roots()[ordinal].exact_root.is_none() {
        let interval = &fiber.roots()[ordinal];
        let count = hypersolve::count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            topology.graph_equation(),
            hypersolve::CurveResultantParameter::First,
            fiber.parameter(),
            &interval.lower,
            &interval.upper,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            count.status,
            hypersolve::AlgebraicFiberRootCountStatus::Counted
        );
        assert_eq!(count.distinct_root_count, Some(1));
        assert_eq!(count.certainty, hyperlimit::Certainty::Exact);
    }
}

fn attachments(t: &BivariateCurveTopology, ordinal: usize) -> Vec<(usize, usize)> {
    let i = t.vertical_components()[ordinal].event_index();
    (0..t.events()[i].roots().len())
        .map(|r| {
            let left = i.checked_sub(1).map_or(0, |s| {
                t.slabs()[s]
                    .arcs()
                    .iter()
                    .filter(|a| a.right_root() == r)
                    .count()
            });
            let right = t.slabs().get(i).map_or(0, |s| {
                s.arcs().iter().filter(|a| a.left_root() == r).count()
            });
            (left, right)
        })
        .collect()
}

pub fn pure_vertical_components_keep_closed_bounds_and_powers() {
    for root in [-2, -1, 0, 1, 2] {
        for exponent in 1..=4 {
            let p = power(&polynomial(&[&[-root], &[1]]), exponent);
            let t = build(&p, &rectangle());
            assert_eq!(t.vertical_components().len(), 1);
            let v = &t.vertical_components()[0];
            assert_eq!(v.defining_exponent(), exponent);
            parameter_is(t.events()[v.event_index()].parameter(), &Real::from(root));
            assert!(t.events().iter().all(|e| e.roots().is_empty()));
            assert!(t.slabs().iter().all(|s| s.arcs().is_empty()));
            assert_eq!(
                t.rational_fiber(&root.into(), Default::default()),
                Err(BivariateTopologyError::VerticalComponent)
            );
            let x = if root == 0 { Real::one() } else { Real::zero() };
            assert!(
                t.rational_fiber(&x, Default::default())
                    .unwrap()
                    .roots()
                    .is_empty()
            );
        }
    }
}

pub fn vertical_crossings_split_regular_residual_branches() {
    let line = polynomial(&[&[0, 1]]);
    let x = polynomial(&[&[0], &[1]]);
    // x^2 * y^3: defining powers and oriented graph incidence are separate.
    let t = build(
        &bivariate_multiply(&power(&x, 2), &power(&line, 3)),
        &rectangle(),
    );
    assert_eq!(t.vertical_components()[0].defining_exponent(), 2);
    assert_eq!(attachments(&t, 0), [(1, 1)]);
    assert_eq!(
        t.repeated_factorization().unwrap().factors()[0].exponent(),
        3
    );
    equal_polynomials(t.graph_equation(), &line);
    equal_polynomials(t.geometric_equation(), &bivariate_multiply(&x, &line));
    let c = t.vertical_components()[0].event_index();
    fiber_root_is(&t, c, 0, &Real::zero());
    // Crossing a vertical component splits a regular parabola even though
    // that first coordinate was not previously a critical/boundary event.
    let parabola = polynomial(&[&[0, 1], &[], &[-1]]);
    let shifted = polynomial(&[&[-1], &[1]]);
    let t = build(&bivariate_multiply(&shifted, &parabola), &rectangle());
    assert_eq!(attachments(&t, 0), [(1, 1)]);
    let c = t.vertical_components()[0].event_index();
    fiber_root_is(&t, c, 0, &Real::one());
}

pub fn vertical_singular_and_boundary_contacts_retain_all_half_branches() {
    let x = polynomial(&[&[0], &[1]]);
    for (residual, incidence) in [
        (polynomial(&[&[0, 0, 1], &[], &[-1]]), vec![(2, 2)]), // node
        (polynomial(&[&[0, 0, 1], &[], &[], &[-1]]), vec![(0, 2)]), // cusp
        (polynomial(&[&[0, 0, 1], &[], &[1]]), vec![(0, 0)]),  // point on interval
    ] {
        let t = build(&bivariate_multiply(&x, &residual), &rectangle());
        assert_eq!(attachments(&t, 0), incidence);
    }
    let horizontal_boundary = polynomial(&[&[-3, 1]]);
    for (root, incidence) in [(-2, vec![(0, 1)]), (2, vec![(1, 0)])] {
        let t = build(
            &bivariate_multiply(&polynomial(&[&[-root], &[1]]), &horizontal_boundary),
            &rectangle(),
        );
        assert_eq!(attachments(&t, 0), incidence);
        assert_eq!(t.domain().second, [Real::from(-3), Real::from(3)]);
    }
}

pub fn selected_vertical_roots_keep_algebraic_coordinates_and_nearby_columns() {
    let irrational = polynomial(&[&[-2], &[], &[1]]);
    let circle = polynomial(&[&[-3, 0, 1], &[], &[1]]);
    let t = build(
        &bivariate_multiply(&power(&irrational, 3), &circle),
        &rectangle(),
    );
    assert_eq!(t.vertical_components().len(), 2);
    for (ordinal, v) in t.vertical_components().iter().enumerate() {
        assert_eq!(v.defining_exponent(), 3);
        assert!(t.events()[v.event_index()].parameter().is_valid());
        assert!(
            t.events()[v.event_index()]
                .parameter()
                .interval
                .exact_root
                .is_none()
        );
        assert_eq!(attachments(&t, ordinal), [(1, 1), (1, 1)]);
        fiber_root_is(&t, v.event_index(), 0, &Real::from(-1));
        fiber_root_is(&t, v.event_index(), 1, &Real::one());
    }
    // A degree-five selected coordinate is admitted without a rational or
    // radical coordinate representation. Its derivative 5x^4+5 is positive;
    // p(-1)<0<p(0) gives the unique root and its exact negative selector.
    let quintic = polynomial(&[&[5], &[5], &[], &[], &[], &[1]]);
    let t = build(&quintic, &rectangle());
    assert_eq!(t.vertical_components().len(), 1);
    assert_eq!(t.vertical_components()[0].defining_exponent(), 1);
    let event = &t.events()[t.vertical_components()[0].event_index()];
    assert!(event.parameter().is_valid());
    assert!(event.parameter().interval.exact_root.is_none());
    assert!(event.roots().is_empty());
    let mut field = hypersolve::AlgebraicField::new(event.parameter()).unwrap();
    let value = field
        .rational_function(
            event.parameter(),
            vec![5.into(), 5.into(), 0.into(), 0.into(), 0.into(), 1.into()],
            vec![Real::one()],
        )
        .unwrap();
    assert_eq!(field.sign(&value).unwrap(), std::cmp::Ordering::Equal);
    let epsilon = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let p = bivariate_multiply(
        &polynomial(&[&[0], &[1]]),
        &axis(&[-epsilon.clone(), Real::one()]),
    );
    let t = build(&p, &rectangle());
    assert_eq!(t.vertical_components().len(), 2);
    for (v, expected) in t.vertical_components().iter().zip([Real::zero(), epsilon]) {
        parameter_is(t.events()[v.event_index()].parameter(), &expected);
    }
}

pub fn vertical_limits_invalid_inputs_and_budget_retry_remain_explicit() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let equation = bivariate_multiply(&polynomial(&[&[-1], &[1]]), &polynomial(&[&[0, 1]]));
    let d = rectangle();
    assert_eq!(
        with_budget(WorkBudget::ZERO, || decompose_bivariate_curve(
            &equation,
            &d,
            Default::default()
        ))
        .result,
        Err(hypersolve::work_budget::BudgetExceeded {
            kind: hypersolve::work_budget::WorkKind::AlgebraicPromotion,
            limit: 0,
            used: 0,
            requested: 1
        })
    );
    assert!(
        with_budget(
            WorkBudget {
                refinements: 100_000,
                promotions: 10_000
            },
            || decompose_bivariate_curve(&equation, &d, Default::default())
        )
        .result
        .unwrap()
        .is_ok()
    );
    assert_eq!(
        decompose_bivariate_curve(
            &equation,
            &d,
            BivariateTopologyConfig {
                max_axis_degree: 0,
                ..Default::default()
            }
        ),
        Err(BivariateTopologyError::DegreeLimit)
    );
    let mut bad = d.clone();
    bad.second.reverse();
    assert_eq!(
        decompose_bivariate_curve(&equation, &bad, Default::default()),
        Err(BivariateTopologyError::InvalidDomain)
    );
    let nonrational = BivariatePolynomial::new(vec![vec![Real::pi()], vec![Real::one()]]);
    assert_eq!(
        decompose_bivariate_curve(&nonrational, &d, Default::default()),
        Err(BivariateTopologyError::UnsupportedCoefficient)
    );
    assert_eq!(
        decompose_bivariate_curve(&polynomial(&[&[0]]), &d, Default::default()),
        Err(BivariateTopologyError::ZeroPolynomial)
    );
    assert_eq!(
        decompose_bivariate_curve(
            &equation,
            &BivariateCurveDomain {
                first: [Real::zero(), Real::one()],
                second: [Real::zero(), Real::one()]
            },
            Default::default()
        )
        .unwrap()
        .vertical_components()
        .len(),
        1
    );
}

pub fn content_scalars_and_unsupported_algebraic_bounds_remain_explicit() {
    let original = bivariate_multiply(
        &power(&polynomial(&[&[-1], &[1]]), 2),
        &power(&polynomial(&[&[0, 1]]), 3),
    );
    for numerator in [-5, 5] {
        let scale = (Real::from(numerator) / Real::from(7)).unwrap();
        let equation = bivariate_multiply(&BivariatePolynomial::new(vec![vec![scale]]), &original);
        let topology = build(&equation, &rectangle());
        assert_eq!(topology.vertical_components()[0].defining_exponent(), 2);
        assert_eq!(
            topology.repeated_factorization().unwrap().factors()[0].exponent(),
            3
        );
        assert_eq!(attachments(&topology, 0), [(1, 1)]);
    }
    let algebraic = Real::from(2).sqrt().unwrap();
    let mut domain = rectangle();
    domain.first[1] = algebraic.clone();
    assert_eq!(
        decompose_bivariate_curve(&original, &domain, Default::default()),
        Err(BivariateTopologyError::UnsupportedCoefficient)
    );
    let equation = BivariatePolynomial::new(vec![vec![-algebraic], vec![Real::one()]]);
    assert_eq!(
        decompose_bivariate_curve(&equation, &rectangle(), Default::default()),
        Err(BivariateTopologyError::UnsupportedCoefficient)
    );
}
