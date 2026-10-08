#[cfg(test)]
mod finite_conic_split_regression {
    use super::*;

    #[test]
    fn finite_conic_split_retains_zero_intermediate_homogeneous_weight() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        // D(t)=1-3t+3t^2=1/4+3(t-1/2)^2 is positive everywhere.
        // At t=2/3, the first split has weights [1,0,1/3], and its
        // middle homogeneous numerator is (-1/3,-1/3), not an affine point.
        let conic = RationalQuadraticBezier2::try_unit_end_weights(
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 0),
            q(-1, 2),
        )
        .unwrap();
        let expected = CurvePoint2::from(Point2::new(Real::from(2), q(-2, 3)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (general, source) in [
                (
                    true,
                    crate::Curve2::from(RationalBezier2::from(conic.clone())),
                ),
                (false, crate::Curve2::from(conic.clone())),
            ] {
                let Ok(split) = source.split_at(q(2, 3).into(), &policy) else {
                    panic!(
                        "finite conic split must retain its homogeneous result: general={general}"
                    );
                };
                let (left, right) = split.value;
                for point in [left.end(), right.start()] {
                    assert!(matches!(
                        point.coincides_with(&expected, &policy).value,
                        Classification::Decided(true)
                    ));
                }
                assert!(
                    crate::CurvePath2::try_new_with_policy(vec![left.clone(), right], &policy)
                        .is_ok()
                );
                assert!(
                    left.split_at(q(1, 2).into(), &policy).is_ok(),
                    "the exact split result must admit a subsequent cut"
                );
            }
        }
    }
}
