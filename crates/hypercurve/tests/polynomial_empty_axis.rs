//! Empty original polynomial sources remain complete native curve sets.
use hypercurve::PolynomialCurveSet2;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

#[test]
fn empty_axis_topology_enters_the_native_curve_set() {
    let domain = BivariateCurveDomain {
        first: [(-1_i32).into(), 1_i32.into()],
        second: [(-2_i32).into(), 2_i32.into()],
    };
    // Strict positivity or an excluded exact root proves emptiness independently.
    for coefficients in [vec![1, 0, 1], vec![-3, 1], vec![9, -6, 1]] {
        let equation =
            BivariatePolynomial::new(coefficients.into_iter().map(|c| vec![c.into()]).collect());
        let topology = decompose_bivariate_curve(&equation, &domain, Default::default()).unwrap();
        assert!(topology.is_empty());
        assert_eq!(topology.equation(), &equation);
        assert_eq!(topology.domain(), &domain);
        let curves =
            PolynomialCurveSet2::from_topology(topology.clone(), Default::default()).unwrap();
        assert!(curves.is_empty());
        assert!(curves.curves().is_empty());
        assert!(curves.isolated_points().is_empty());
        assert!(
            topology
                .rational_fiber(&0_i32.into(), Default::default())
                .unwrap()
                .roots()
                .is_empty()
        );
        let reconstructed =
            decompose_bivariate_curve(topology.equation(), topology.domain(), Default::default())
                .unwrap();
        assert!(
            PolynomialCurveSet2::from_topology(reconstructed, Default::default())
                .unwrap()
                .is_empty()
        );
    }
}
