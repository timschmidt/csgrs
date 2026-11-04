//! Bounded rational products with independent component and slice censuses.
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_specialize_first};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicField, BivariatePolynomial, UnivariateSturmSequence};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn power(p: &BivariatePolynomial, n: usize) -> BivariatePolynomial {
    (0..n).fold(polynomial(&[&[1]]), |a, _| bivariate_multiply(&a, p))
}

pub fn check(data: &[u8]) {
    if data.len() < 5 {
        return;
    }
    if data[4] % 6 == 5 {
        check_quintic(data);
        return;
    }
    let root = |byte: u8| (Real::from(i32::from(byte & 7) - 3) / Real::from(4)).unwrap();
    let roots = [root(data[0]), root(data[1])];
    let exponents = [usize::from(data[2] % 3) + 1, usize::from(data[3] % 3) + 1];
    let linear = |r: &Real| BivariatePolynomial::new(vec![vec![-r], vec![Real::one()]]);
    let content = bivariate_multiply(
        &power(&linear(&roots[0]), exponents[0]),
        &power(&linear(&roots[1]), exponents[1]),
    );
    let residual = match data[4] % 6 {
        0 => polynomial(&[&[1]]),
        1 => polynomial(&[&[0, 1]]),
        2 => polynomial(&[&[-1, 0, 1], &[], &[1]]), // circle
        3 => polynomial(&[&[0, 0, 1], &[], &[-1]]), // node
        _ => polynomial(&[&[0, 0, 1], &[], &[], &[-1]]), // cusp
    };
    let equation = bivariate_multiply(&content, &residual);
    let domain = BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-2).into(), 2.into()],
    };
    let topology = decompose_bivariate_curve(
        &equation,
        &domain,
        BivariateTopologyConfig {
            max_axis_degree: 12,
            ..Default::default()
        },
    )
    .expect("bounded admitted rational component fixture must complete");
    assert!(!topology.is_empty());
    assert_eq!(topology.equation(), &equation);
    assert_eq!(
        topology.vertical_components().len(),
        if roots[0] == roots[1] { 1 } else { 2 }
    );
    for component in topology.vertical_components() {
        let event = &topology.events()[component.event_index()];
        assert!(event.parameter().is_valid());
        let mut field = AlgebraicField::new(event.parameter()).unwrap();
        let mut expected_exponent = 0;
        for (root, exponent) in roots.iter().zip(exponents) {
            let value = field
                .rational_function(
                    event.parameter(),
                    vec![-root, Real::one()],
                    vec![Real::one()],
                )
                .unwrap();
            if field.sign(&value).unwrap() == Ordering::Equal {
                expected_exponent += exponent;
            }
        }
        assert!(expected_exponent > 0);
        assert_eq!(component.defining_exponent(), expected_exponent);
        let root = roots
            .iter()
            .find(|root| {
                let value = field
                    .rational_function(
                        event.parameter(),
                        vec![-*root, Real::one()],
                        vec![Real::one()],
                    )
                    .unwrap();
                field.sign(&value).unwrap() == Ordering::Equal
            })
            .unwrap();
        let p = bivariate_specialize_first(&residual, root);
        let sturm = UnivariateSturmSequence::new(&p, PredicatePolicy::STRICT).unwrap();
        let lower_root = Real::eval_poly(&p, &domain.second[0]).definitely_zero();
        let count = sturm
            .count_distinct_roots(
                &domain.second[0],
                &domain.second[1],
                PredicatePolicy::STRICT,
            )
            .unwrap()
            + usize::from(lower_root);
        assert_eq!(event.roots().len(), count);
    }
}

fn check_quintic(data: &[u8]) {
    let factor = polynomial(&[&[5], &[5], &[], &[], &[], &[1]]);
    let exponent = usize::from(data[2] % 2) + 1;
    let residual = if data[3].is_multiple_of(2) {
        polynomial(&[&[0, 1]])
    } else {
        polynomial(&[&[0, 0, 1], &[1]])
    };
    let equation = bivariate_multiply(&power(&factor, exponent), &residual);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-2).into(), 2.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig {
            max_axis_degree: 12,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(topology.equation(), &equation);
    // p'=5x^4+5>0 and p(-1)<0<p(0): exactly one vertical component.
    assert_eq!(topology.vertical_components().len(), 1);
    let v = &topology.vertical_components()[0];
    assert_eq!(v.defining_exponent(), exponent);
    let event = &topology.events()[v.event_index()];
    assert!(event.parameter().is_valid());
    assert!(event.parameter().interval.exact_root.is_none());
    // y=0 has one root. Since -1<alpha<0, y^2+alpha has two
    // roots strictly within [-2,2], independently of fiber isolation.
    assert_eq!(
        event.roots().len(),
        if data[3].is_multiple_of(2) { 1 } else { 2 }
    );
    let mut field = AlgebraicField::new(event.parameter()).unwrap();
    let value = field
        .rational_function(
            event.parameter(),
            vec![5.into(), 5.into(), 0.into(), 0.into(), 0.into(), 1.into()],
            vec![Real::one()],
        )
        .unwrap();
    assert_eq!(field.sign(&value).unwrap(), Ordering::Equal);
}
