    fn recursive_test_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        let fifth = (Real::one() / Real::from(5)).unwrap();
        let start = point.translated(Real::from(4) * &fifth, -Real::from(3) * &fifth);
        let end = start.translated(Real::from(3), Real::from(4));
        let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(start.into(), end.into(), policy).unwrap() else {
            panic!("the independent oblique source chord is nondegenerate");
        };
        let (point, _) = BezierAlgebraicChordParallelPoint2::new_pair(chord, Real::one(), Real::zero(), Real::zero(), policy);
        let point = CurvePoint2::from(point);
        assert!(point.coordinates().is_none());
        point
    }

    #[test]
    fn recursive_incidence_preserves_selected_point_fields() {
        let fifth = (Real::one() / Real::from(5)).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = QuadraticBezier2::new(Point2::from_values(0, 0), Point2::new(half.clone(), Real::zero()), Point2::from_values(1, 0)).parallel_left(-Real::one()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let BezierParameter2::Algebraic(alpha) = algebraic_parameter(vec![-half.clone(), Real::zero(), Real::one()]) else { unreachable!() };
            for sign in [-1, 1] {
                let image = |x: Real, y: Real| CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_retained_expression(
                    alpha.clone(), parameter_representation(&alpha, &policy),
                    vec![x, Real::from(sign)], vec![y], vec![Real::one()],
                    "independent recursive point incidence",
                ));
                let start = image(Real::from(4) * &fifth, -Real::from(8) * &fifth);
                let end = image(Real::from(19) * &fifth, Real::from(12) * &fifth);
                let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(start, end, &policy).unwrap() else { panic!("selected endpoints define the same oblique direction") };
                let (point, _) = BezierAlgebraicChordParallelPoint2::new_pair(chord, Real::one(), Real::zero(), Real::zero(), &policy);
                let point = CurvePoint2::from(point);
                assert!(point.coordinates().is_none());
                assert_eq!(point.coincides_with(&image(Real::zero(), -Real::one()), &policy).value, Classification::Decided(true));
                for reversed in [false, true] {
                    for (start, end, matches) in [(0, 1, sign > 0), (-1, 0, sign < 0)] {
                        let range = if reversed { CurveParameterRange2::new_validated(Real::from(end).into(), Real::from(start).into()) } else { CurveParameterRange2::new_validated(Real::from(start).into(), Real::from(end).into()) };
                        let mut parameters = Vec::new();
                        assert_eq!(parallel.visit_point_incidence_evidence(&point, &range, None, false, &policy, &mut |parameter| {
                            parameters.push(parameter.expect("the affine source has an isolated preimage").clone());
                            ControlFlow::Continue(())
                        }).unwrap(), Classification::Decided(ControlFlow::Continue(())));
                        assert_eq!(parameters.len(), usize::from(matches));
                        for parameter in parameters {
                            assert_eq!(parameter.polynomial_sign(&[-half.clone(), Real::zero(), Real::one()], &policy).unwrap(), Classification::Decided(RealSign::Zero));
                            assert_eq!(parameter.cmp_by_refinement(&Real::zero().into(), &policy).unwrap(), Classification::Decided(if sign > 0 { std::cmp::Ordering::Greater } else { std::cmp::Ordering::Less }));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn recursive_incidence_keeps_stationary_and_collapsed_semantics() {
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let parallel = QuadraticBezier2::new(Point2::from_values(1, 0), Point2::from_values(-1, 0), Point2::from_values(1, 0)).parallel_left(Real::one()).unwrap();
        let center = Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap());
        let circle = RationalQuadraticBezier2::try_unit_end_weights(
            center.translated(Real::one(), Real::zero()), center.translated(Real::one(), Real::one()), center.translated(Real::zero(), Real::one()),
            (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
        ).unwrap().parallel_left(Real::one()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (y, expected) in [(1, Real::from(3) * &quarter), (-1, quarter.clone())] {
                let point = recursive_test_point(&Point2::new(quarter.clone(), Real::from(y)), &policy);
                let mut parameters = Vec::new();
                assert_eq!(parallel.visit_point_incidence_evidence(&point, &CurveParameterRange2::unit(), None, false, &policy, &mut |parameter| {
                    parameters.push(parameter.unwrap().clone()); ControlFlow::Continue(())
                }).unwrap(), Classification::Decided(ControlFlow::Continue(())));
                assert_eq!(parameters.len(), 1);
                assert_eq!(parameters[0].same_value(&expected.into(), &policy).unwrap(), Classification::Decided(true));
            }
            let point = recursive_test_point(&Point2::from_values(0, 1), &policy);
            assert!(matches!(parallel.contains_point_evidence(&point, &CurveParameterRange2::unit(), None, &policy).unwrap(), Classification::Uncertain(UncertaintyReason::Boundary)));
            let point = recursive_test_point(&Point2::from_values(0, 0), &policy);
            assert_eq!(parallel.with_distance(Real::zero()).contains_point_evidence(&point, &CurveParameterRange2::unit(), None, &policy).unwrap(), Classification::Decided(true));
            let point = recursive_test_point(&center, &policy);
            for (distance, expected) in [(1, 1), (-1, 0)] {
                let mut visits = 0;
                assert_eq!(circle.with_distance(Real::from(distance)).visit_point_incidence_evidence(&point, &CurveParameterRange2::unit(), None, false, &policy, &mut |parameter| {
                    assert!(parameter.is_none(), "a collapsed source retains the entire contact domain");
                    visits += 1; ControlFlow::Continue(())
                }).unwrap(), Classification::Decided(ControlFlow::Continue(())));
                assert_eq!(visits, expected);
            }
        }
    }

    #[test]
    fn recursive_incidence_preserves_exterior_domains_and_pole_barriers() {
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let parallel = RationalBezier2::try_new(vec![Point2::from_values(0, 0), Point2::from_values(1, 0)], vec![-Real::one(), Real::one()]).unwrap().parallel_left(Real::one()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point = recursive_test_point(&Point2::new(Real::from(3) * &quarter, -Real::one()), &policy);
            let foreign = recursive_test_point(&Point2::new(quarter.clone(), -Real::one()), &policy);
            for reversed in [false, true] {
                let range = if reversed { CurveParameterRange2::new_validated(Real::from(3).into(), Real::from(2).into()) } else { CurveParameterRange2::new_validated(Real::from(2).into(), Real::from(3).into()) };
                let Classification::Decided(incident) = parallel.incident_domain_from_parameter(&Real::from(2).into(), BezierParameterRayDirection2::Decreasing, &policy).unwrap() else { panic!("the exterior source component has a first pole") };
                assert_eq!(parallel.contains_point_evidence(&point, &range, None, &policy).unwrap(), Classification::Decided(false));
                assert_eq!(parallel.contains_point_evidence(&point, &range, Some(&incident), &policy).unwrap(), Classification::Decided(true));
                assert_eq!(parallel.contains_point_evidence(&foreign, &range, Some(&incident), &policy).unwrap(), Classification::Decided(false));
            }
            assert!(matches!(parallel.contains_point_evidence(&point, &CurveParameterRange2::unit(), None, &policy).unwrap(), Classification::Uncertain(UncertaintyReason::Boundary)));
        }
    }
