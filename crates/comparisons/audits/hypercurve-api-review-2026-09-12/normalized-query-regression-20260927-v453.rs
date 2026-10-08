
    #[test]
    fn nonrational_query_quotients_preserve_tiny_signs_on_repeated_conjugates() {
        let inner = Real::from(5).sqrt().unwrap();
        let outer = (Real::from(50) - Real::from(20) * &inner).sqrt().unwrap();
        let unit = inner + outer;
        let wide = Real::from(2).powi_i64(256).unwrap();
        let tiny = Real::from(2).powi_i64(-256).unwrap();
        // (x^2 - 2)^2 owns one repeated root in each interval. A multiple
        // of this defining polynomial vanishes there, however large its
        // coefficients become compared with the retained linear query.
        let base = [4, 0, -4, 0, 1];
        for orientation in [-1_i32, 1] {
            let defining = base.map(|coefficient| Real::from(coefficient * orientation) * &unit);
            let quotient = [&wide * (&unit + Real::one()), -&wide * &unit, Real::zero(), &wide * &unit];
            let mut multiple = vec![Real::zero(); defining.len() + quotient.len() - 1];
            for (i, a) in defining.iter().enumerate() {
                for (j, b) in quotient.iter().enumerate() { multiple[i + j] += a * b; }
            }
            for (lower, upper, root_sign) in [(-2, -1, -1_i32), (1, 2, 1)] {
                let interval = IsolatedRootInterval { lower: Real::from(lower), upper: Real::from(upper), exact_root: None, distinct_root_count: 1 };
                for query_sign in [-1_i32, 0, 1] {
                    let mut predicate = multiple.clone();
                    predicate[1] += Real::from(query_sign) * &tiny * &unit;
                    let expected = (query_sign * root_sign).cmp(&0);
                    assert_eq!(sign_at_selected_root(&defining, &predicate, &interval), Some(expected));
                }
            }
        }
    }

    #[test]
    fn zero_query_does_not_require_an_unused_leading_degree_decision() {
        let epsilon = Real::one() - Real::from(2).powi_i64(-600).unwrap().cos();
        // epsilon >= 0, so this polynomial is strictly increasing on [0,1]
        // with opposite endpoint signs, even if epsilon's scalar sign is not
        // available to the decision cascade. The zero query needs no degree.
        let defining = [Real::from(-1), Real::from(2), epsilon];
        let interval = IsolatedRootInterval { lower: Real::zero(), upper: Real::one(), exact_root: None, distinct_root_count: 1 };
        assert_eq!(sign_at_selected_root(&defining, &vec![Real::zero(); 8], &interval), Some(Ordering::Equal));
    }
