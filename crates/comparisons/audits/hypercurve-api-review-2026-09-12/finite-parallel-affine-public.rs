use hypercurve::*;

fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected exact evaluation: {reason:?}"),
    }
}

fn main() {
    let half = (Real::one() / Real::from(2)).unwrap();
    let source = QuadraticBezier2::new(
        Point2::from_values(0, 4),
        Point2::new(half.clone(), Real::from(2)),
        Point2::from_values(1, 1),
    );
    let delta = (Real::from(4).root_n(3).unwrap() - Real::one()).sqrt().unwrap() * half;
    let mut sources = vec![BezierParallelSource2::Quadratic(source.clone())];
    for gauge in [1, -3] {
        let rational = RationalBezier2::try_new(
            source.control_points().into_iter().cloned().collect(),
            vec![Real::from(gauge); 3],
        ).unwrap();
        sources.push(BezierParallelSource2::Rational(rational.clone()));
        sources.push(BezierParallelSource2::Rational(rational.elevated_to_degree(5).unwrap()));
    }
    let mut checks = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (family, source) in sources.iter().enumerate() {
            for distance in [-1, 1] {
                let parallel = BezierParallel2::from_source(source.clone(), Real::from(distance));
                for t in [Real::from(2)-&delta, Real::from(2)+&delta] {
                    for reverse in [false, true] {
                        let (parallel, parameter) = if reverse {
                            (parallel.reversed(), Real::one()-&t)
                        } else { (parallel.clone(), t.clone()) };
                        let derivative = decided(parallel.derivative_at(&parameter, &policy).unwrap());
                        let expected = if distance == 1 {
                            assert_eq!(derivative.zero_status(), hypercurve::Real::zero().zero_status(), "cusp family={family}, reverse={reverse}");
                            Point2::from_values(0, 0)
                        } else {
                            let direction = Real::from(if reverse { -1 } else { 1 });
                            Point2::new(Real::from(2)*&direction, Real::from(4)*(t.clone()-Real::from(2))*direction)
                        };
                        let actual = Point2::new(derivative.dx().clone(), derivative.dy().clone());
                        let comparison = CurvePoint2::from(actual).coincides_with(&expected.into(), &policy);
                        assert_eq!(comparison.certainty, CurveCertainty::Certified);
                        assert_eq!(comparison.into_value(), Classification::Decided(true), "family={family}, distance={distance}, reverse={reverse}");
                        checks += 1;
                    }
                }
            }
        }
    }
    println!("{{\"cusp_representations\":{},\"checks\":{checks}}}", sources.len());
}
