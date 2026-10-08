    #[test]
    fn parallel_derivative_orientation_replays_unprojectable_selected_fibers() {
        let p = Point2::from_values;
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))
            .parallel_left(half.clone()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // u^135 = 1/(2*scale^9). The first root lies in (0.99,1),
            // the second in (0.49,0.5), on opposite sides of a support cusp.
            for (scale, expected) in [(1, RealSign::Positive), (32_768, RealSign::Negative)] {
                let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                    half.clone(), scale, &policy,
                );
                assert!(matches!(selected.promoted_bezier_parameter(&policy).unwrap(), Classification::Uncertain(_)));
                let parameter = CurveParameter2::from_selected_fiber(selected);
                assert_eq!(parallel.parallel_derivative_scale_sign(&parameter, &policy).unwrap(),
                           Classification::Decided(expected));
            }
        }
    }
