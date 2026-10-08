    #[test]
    fn chord_parallel_monotonicity_covers_the_retained_parameter_range() {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(t,15t/4-2t²+t³/3), D=(1,0). The cross polynomial
        // (t-3/2)(t-5/2) is positive on [0,1] and at both ends of
        // [1,3], but has two interior roots in that extended range.
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(1, 3), q(5, 4)),
            Point2::new(q(2, 3), q(11, 6)),
            Point2::new(Real::one(), q(25, 12)),
        );
        let mut parallels = vec![source.parallel_left(Real::zero()).unwrap()];
        for gauge in [1, -3] {
            parallels.push(
                RationalBezier2::try_new(
                    source.control_points().into_iter().cloned().collect(),
                    vec![Real::from(gauge); 4],
                )
                .unwrap()
                .parallel_left(Real::zero())
                .unwrap(),
            );
        }
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(0, 0)),
                CurvePoint2::from(Point2::from_values(1, 0)),
                &policy,
            )
            .unwrap() else {
                panic!("the exact horizontal chord must construct");
            };
            for (chord, orientation) in [
                (chord.clone(), RealSign::Positive),
                (chord.reversed(), RealSign::Negative),
            ] {
                for parallel in &parallels {
                    for (lower, upper, expected) in [
                        (Real::one(), Real::from(3), None),
                        (Real::zero(), Real::one(), Some(RealSign::Positive)),
                        (Real::from(3), Real::from(4), Some(RealSign::Positive)),
                        (q(7, 4), q(9, 4), Some(RealSign::Negative)),
                        // Roots at the two endpoints do not invalidate a
                        // strict sign in the open retained interval.
                        (q(3, 2), q(5, 2), Some(RealSign::Negative)),
                    ] {
                        for (start, end) in [
                            (lower.clone(), upper.clone()),
                            (upper, lower),
                        ] {
                            let range = CurveParameterRange2::new_validated(
                                BezierParameter2::Exact(start).into(),
                                BezierParameter2::Exact(end).into(),
                            );
                            let result = chord
                                .parallel_tangent_cross_sign_on_region_range(
                                    parallel, &range, &policy,
                                )
                                .unwrap();
                            match expected {
                                None => assert!(
                                    matches!(result, Classification::Uncertain(_)),
                                    "two interior tangent roots forbid a monotonicity certificate: {result:?}",
                                ),
                                Some(sign) => assert_eq!(
                                    result,
                                    Classification::Decided(product_sign(sign, orientation)),
                                ),
                            }
                        }
                    }
                }
            }
        }
    }

