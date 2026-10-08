    #[test]
    fn rational_sturm_preserves_selected_multiplicity_and_negative_scales() {
        // F(t,u)=u^3+t*u+1 has two distinct roots at
        // t=-cbrt(27/4): beta=cbrt(1/2) is double and -2*beta is simple.
        // The same reducible defining polynomial also owns t=5, where F is
        // strictly increasing and has exactly one real root.
        let defining = vec![real(-135), real(27), real(0), real(-20), real(4)];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (lower, upper, count, length) in [(-2, -1, 2, 3), (4, 6, 1, 4)] {
                let root = represented_root(defining.clone(), real(lower), real(upper), policy);
                for scale in [Real::one(), rational(-3, 7)] {
                    let fiber = BivariatePolynomial::new(vec![
                        vec![scale.clone(), real(0), real(0), scale.clone()],
                        vec![real(0), scale],
                    ]);
                    let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                    let local = local_fiber_polynomial(&fiber, CurveResultantParameter::First, &mut field).unwrap();
                    assert!(rational_local_sturm_rows(&local, &mut field).is_some());
                    let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                        &fiber, CurveResultantParameter::First, &root, &real(-3), &real(3), policy,
                    );
                    assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                    assert_eq!(report.distinct_root_count, Some(count));
                    assert_eq!(report.sturm_sequence_length, length);
                    assert_eq!(report.certainty, Certainty::Exact);
                }
            }
        }
    }

    #[test]
    fn rational_sturm_declines_degree_changes_and_general_exact_coefficients() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let sqrt_three = real(3).sqrt().unwrap();
            let cases = [
                // F=u^4+u+t has one critical point. F(0)<0 and F(+/-2)>0
                // at t=-sqrt(2), so there are exactly two real roots.
                (vec![real(-2), real(0), real(1)], real(-2), real(-1),
                 BivariatePolynomial::new(vec![vec![real(0),real(1),real(0),real(0),real(1)],vec![real(1)]]),2),
                // At t=sqrt(3), the linear PRS row loses only its leading
                // term. F=u*(u^2+t*u+1) has only the simple root zero.
                (vec![real(-3), real(0), real(1)], real(1), real(2),
                 BivariatePolynomial::new(vec![vec![real(0),real(1),real(0),real(1)],vec![real(0),real(0),real(1)]]),1),
                // Arbitrary exact Real coefficients remain on their exact
                // field path: (u-sqrt(3))*(u+1) has two represented roots.
                (vec![real(-2), real(0), real(1)], real(1), real(2),
                 BivariatePolynomial::new(vec![vec![-sqrt_three.clone(),Real::one()-&sqrt_three,Real::one()]]),2),
            ];
            for (defining, lower, upper, fiber, count) in cases {
                let root = represented_root(defining, lower, upper, policy);
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                let local = local_fiber_polynomial(&fiber, CurveResultantParameter::First, &mut field).unwrap();
                assert!(rational_local_sturm_rows(&local, &mut field).is_none());
                let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                    &fiber, CurveResultantParameter::First, &root, &real(-2), &real(2), policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                assert_eq!(report.distinct_root_count, Some(count));
                assert_eq!(report.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn rational_sturm_isolates_degree_41_fillet_contact_in_both_orientations() {
        // A radius-2/5 fillet between two non-PH quadratic Beziers, followed
        // by Boolean normalization, retains this exact circle/curve fiber.
        // Its one contact in [0,1] is repeated; the defining polynomial also
        // contains degree-2 and degree-5 factors outside its selected isolator.
        let data: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/data/nonph_fillet_circle_contact.json"
        )).unwrap();
        let read = |value: &serde_json::Value| Real::new(value.as_str().unwrap().parse::<hyperreal::Rational>().unwrap());
        let coefficients = data["incidence"].as_array().unwrap().iter().map(|row| {
            row.as_array().unwrap().iter().map(&read).collect::<Vec<_>>()
        }).collect::<Vec<_>>();
        let mut transposed = vec![vec![Real::zero(); coefficients.len()]; coefficients.iter().map(Vec::len).max().unwrap()];
        for (i,row) in coefficients.iter().enumerate() {
            for (j,value) in row.iter().enumerate() { transposed[j][i]=value.clone(); }
        }
        let fibers = [
            (BivariatePolynomial::new(coefficients), CurveResultantParameter::First),
            (BivariatePolynomial::new(transposed), CurveResultantParameter::Second),
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let root = represented_root(
                data["base"].as_array().unwrap().iter().map(&read).collect(),
                read(&data["base_interval"][0]), read(&data["base_interval"][1]), policy,
            );
            for (fiber, orientation) in &fibers {
                let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                    fiber, *orientation, &root, &Real::zero(), &Real::one(),
                    AlgebraicFiberRootIsolationConfig { max_subdivision_depth: 512, refinement_steps: 8 }, policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
                assert_eq!(report.intervals.len(), 1);
                assert_eq!(report.intervals[0].distinct_root_count, 1);
                assert!(report.intervals[0].lower >= Real::zero());
                assert!(report.intervals[0].upper <= Real::one());
                assert_eq!(report.sturm_sequence_length, 8);
                assert_eq!(report.certainty, Certainty::Exact);
            }
        }
    }

