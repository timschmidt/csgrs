
    // The barrier and anchor have distinct exact representations of sqrt(2).
    // Empty-ray classification must reuse the isolated root's certificate.
    #[test]
    fn empty_incident_chart_compares_an_algebraic_barrier_exactly() {
        use CurveParameterComponentSelection2::{NeedsConstraint, Selected};
        let diagonal = BivariatePolynomial::new(vec![
            vec![Real::zero(), -Real::one()], vec![Real::one()],
        ]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-2_i8), Real::zero(), Real::one()], &policy,
            ));
            let interval = decided(BezierParameterInterval::try_new(
                Real::one(), Real::from(2_i8), &policy,
            ));
            let barrier = BezierParameter2::Algebraic(decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy),
            ));
            let anchor = Real::from(2_i8).sqrt().unwrap();
            let parameter = CurveParameter2::from(barrier.clone());
            let finite = CurveParameterRange2::new_validated(
                Real::one().into(), Real::from(2_i8).into(),
            );
            for direction in [BezierParameterRayDirection2::Increasing, BezierParameterRayDirection2::Decreasing] {
                let ray = BezierParameterRay2 { anchor: &anchor, direction, barrier: Some(&barrier) };
                let result = decided(select_parameter_component_in_domain(
                    &diagonal, &ParameterComponentSelector2::Positive(&positive, None),
                    [CurveParameterDomain2::new(&finite, Some(ray)); 2],
                    ParameterComponentQuery2::AllComponents(None), &policy, config,
                ));
                assert_eq!(result.components.len(), 1);
                let component = &result.components[0];
                assert!(matches!(decided(component.constrain([None, None], &policy)), NeedsConstraint));
                let Selected(pair) = decided(component.constrain([Some(&parameter), None], &policy)) else {
                    panic!("an exact algebraic contact must select the finite component");
                };
                for actual in pair {
                    assert!(decided(actual.same_value(&parameter, &policy)));
                }
            }
        }
    }
