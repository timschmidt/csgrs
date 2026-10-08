    #[test]
    fn represented_root_imports_admit_exact_values_in_the_requested_domain() {
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let values = [
            (-Real::one(), false),
            (Real::from(2), false),
            (-Real::from(2).sqrt().unwrap(), false),
            (Real::from(2).sqrt().unwrap(), false),
            (Real::pi() * &quarter, true),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (value, in_unit_domain) in &values {
                for form in 0..3 {
                    let mut representation = AlgebraicRootRepresentation::from_exact_value(value);
                    if form != 0 {
                        // The solver owns (lower, upper]. Its lower endpoint
                        // is a different root, which the importer must deflate.
                        // Form 1 owns value at the upper endpoint; form 2 owns
                        // the same root strictly inside the deflated interval.
                        let lower = value - Real::one();
                        representation.polynomial_coefficients = vec![
                            &lower * value, -(&lower + value), Real::one(),
                        ];
                        representation.interval = hypersolve::IsolatedRootInterval {
                            lower,
                            upper: if form == 1 { value.clone() } else { value + &quarter },
                            exact_root: None,
                            distinct_root_count: 1,
                        };
                        representation.validation = hypersolve::validate_algebraic_root_representation(
                            &representation, hypersolve::PredicatePolicy::STRICT,
                        );
                    }
                    assert!(representation.is_valid());
                    let generic = BezierParameter2::from_algebraic_root_representation_unbounded(
                        &representation, &policy,
                    ).unwrap();
                    let Classification::Decided(BezierParameter2::Exact(actual)) = generic else {
                        panic!("an explicitly represented root retains its exact scalar");
                    };
                    assert_eq!(compare_reals(&actual, value, &CurveContext::STRICT), Some(std::cmp::Ordering::Equal));
                    let native = BezierParameter2::from_algebraic_root_representation(&representation, &policy);
                    if *in_unit_domain {
                        let Ok(Classification::Decided(BezierParameter2::Exact(actual))) = native else {
                            panic!("the native chart admits the selected interior value");
                        };
                        assert_eq!(compare_reals(&actual, value, &CurveContext::STRICT), Some(std::cmp::Ordering::Equal));
                    } else {
                        assert!(matches!(native, Err(CurveError::InvalidBezierParameter)));
                    }
                }
            }
        }
    }
