    #[test]
    fn certified_newton_signs_overcome_large_field_cancellation() {
        let inner = Real::from(5).sqrt().unwrap();
        let unit = &inner + (Real::from(50) - Real::from(20) * &inner).sqrt().unwrap();
        let wide = Real::from(2).powi_i64(256).unwrap();
        let tiny = Real::from(2).powi_i64(-600).unwrap();
        let outer = Real::from(2).sqrt().unwrap();
        assert!(outer.exact_rational_ref().is_none());
        for root_sign in [-1, 1] {
            let interval = if root_sign < 0 {
                IsolatedRootInterval {
                    lower: -&outer,
                    upper: -Real::one(),
                    exact_root: None,
                    distinct_root_count: 1,
                }
            } else {
                IsolatedRootInterval {
                    lower: Real::one(),
                    upper: outer.clone(),
                    exact_root: None,
                    distinct_root_count: 1,
                }
            };
            for orientation in [-1, 1] {
                // P = +/- unit * (x^3 -/+ 2) selects either real cube root.
                // Q = 2^256 P + 2^-600 x has the selected root's sign.
                // The nonrational endpoint is an enclosure, not a payload
                // reconstruction request; the original field and root stay owned.
                let defining = [-2 * root_sign, 0, 0, 1]
                    .map(|coefficient| Real::from(coefficient * orientation) * &unit);
                let mut predicate = defining.each_ref().map(|value| value * &wide);
                predicate[1] += &tiny;
                let expected = Some(root_sign.cmp(&0));
                assert_eq!(sign_on_refined_singleton(&defining, &predicate, &interval), expected);
                assert_eq!(sign_at_selected_root(&defining, &predicate, &interval), expected);
            }
        }
    }

    #[test]
    fn newton_uncertainty_retains_zero_and_repeated_root_proofs() {
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        let query = [-2, 0, 1].map(Real::from);
        for defining in [query.to_vec(), [4, 0, -4, 0, 1].map(Real::from).to_vec()] {
            assert_eq!(sign_on_refined_singleton(&defining, &query, &interval), None);
            assert_eq!(sign_at_selected_root(&defining, &query, &interval), Some(Ordering::Equal));
        }
        let repeated = [4, 0, -4, 0, 1].map(Real::from);
        let positive = [-1, 1].map(Real::from);
        assert_eq!(sign_on_refined_singleton(&repeated, &positive, &interval), None);
        assert_eq!(sign_at_selected_root(&repeated, &positive, &interval), Some(Ordering::Greater));
    }

    #[test]
    fn dyadic_filter_budget_does_not_limit_exact_query_precision() {
        let defining = [-2, 0, 0, 1].map(Real::from);
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        for orientation in [-1, 1] {
            let predicate = [
                Real::zero(),
                Real::from(orientation) * Real::from(2).powi_i64(-4096).unwrap(),
            ];
            assert_eq!(sign_on_refined_singleton(&defining, &predicate, &interval), None);
            assert_eq!(sign_at_selected_root(&defining, &predicate, &interval), Some(orientation.cmp(&0)));
        }
    }

