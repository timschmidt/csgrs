    fn check_algebraic_image_collapsed_parallel_domain(recursive: bool) {
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = RationalQuadraticBezier2::try_unit_end_weights(
            Point2::from_values(2, 0), Point2::from_values(2, 1), Point2::from_values(1, 1),
            Real::from(2).sqrt().unwrap() * &half,
        ).unwrap().parallel_left(Real::one()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let BezierParameter2::Algebraic(alpha) = algebraic_parameter(vec![-half.clone(), Real::zero(), Real::one()]) else { unreachable!() };
            // Q(alpha)=(2*alpha^2,0)=(1,0), independently of the circle.
            let point = CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_retained_expression(
                alpha.clone(), parameter_representation(&alpha, &policy),
                vec![Real::zero(), Real::zero(), Real::from(2)], vec![Real::zero()], vec![Real::one()],
                "independent algebraic image at a collapsed parallel",
            ));
            assert!(point.coordinates().is_none());
            let same = point.coincides_with(&CurvePoint2::from(Point2::from_values(1, 0)), &policy);
            assert_eq!(same.certainty, crate::CurveCertainty::Certified);
            assert_eq!(same.value, Classification::Decided(true));
            for (distance, expected) in [(1, 1), (-1, 0)] {
                let mut visits = 0;
                let parallel = parallel.with_distance(Real::from(distance));
                let range = CurveParameterRange2::unit();
                let mut visitor = |parameter: Option<&CurveParameter2>| {
                    assert!(parameter.is_none(), "a collapsed parallel retains all source parameters");
                    visits += 1; ControlFlow::Continue(())
                };
                let result = if recursive {
                    BezierParallelPointQuery2 { parallel: &parallel, range: &range, frame: None }
                        .visit_recursive_point_parameters(&point, None, CurveParameterDomain2::new(&range, None), &policy, &mut visitor)
                } else {
                    parallel.visit_point_incidence_evidence(&point, &range, None, false, &policy, &mut visitor)
                };
                assert_eq!(result.unwrap(), Classification::Decided(ControlFlow::Continue(())));
                assert_eq!(visits, expected);
            }
        }
    }

    #[test]
    fn algebraic_image_incidence_retains_a_collapsed_parallel_domain() {
        check_algebraic_image_collapsed_parallel_domain(false);
    }

    #[test]
    fn retained_field_incidence_retains_a_collapsed_parallel_domain() {
        check_algebraic_image_collapsed_parallel_domain(true);
    }
