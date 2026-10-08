
#[cfg(test)]
mod finite_root_envelope_regression {
    use super::*;

    #[test]
    fn rational_isolation_envelopes_retain_exact_endpoint_ownership() {
        // t(t-2)(t^2-2) has the four independently known roots
        // -sqrt(2), 0, sqrt(2), and 2. Pi is an unrelated exact boundary.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) = crate::BezierParameterPolynomial::try_new_power_basis(
                [0, 4, -2, -2, 1].into_iter().map(Real::from).collect(),
                &policy,
            ).unwrap() else { panic!("the rational polynomial must be admitted") };
            let root = Real::from(2).sqrt().unwrap();
            for (lower, upper, endpoints, interior) in [
                (-root.clone(), root.clone(), [true, true], Real::zero()),
                (root.clone(), Real::pi(), [true, false], Real::from(2)),
            ] {
                for inclusion in [[true, true], [false, true], [true, false], [false, false]] {
                    for reversed in [false, true] {
                        let (start, end) = if reversed { (&upper, &lower) } else { (&lower, &upper) };
                        let range = CurveParameterRange2::new_validated(start.clone().into(), end.clone().into());
                        let domain = CurveParameterDomain2::new(&range, None).with_finite_inclusion(inclusion);
                        let Classification::Decided(roots) = domain.finite_roots(&polynomial, &policy).unwrap()
                        else { panic!("exact finite boundaries must retain the complete root inventory") };
                        let mut expected = Vec::new();
                        if endpoints[0] && inclusion[0] { expected.push(lower.clone()); }
                        expected.push(interior.clone());
                        if endpoints[1] && inclusion[1] { expected.push(upper.clone()); }
                        assert_eq!(roots.len(), expected.len());
                        for (actual, expected) in roots.iter().zip(expected) {
                            assert_eq!(actual.same_value(&BezierParameter2::Exact(expected), &policy).unwrap(), Classification::Decided(true));
                        }
                    }
                }
            }
        }
    }
}
