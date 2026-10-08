
    #[test]
    fn selected_parallel_normal_circle_accepts_a_center_locus_cusp() {
        let q = |n: i8, d: i8| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(t,t^2) has a regular source normal at t=0, while its
        // left parallel at distance 1/2 has zero derivative there.
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            Point2::from_values(1, 1),
        );
        let center_support = source.parallel_left(q(1, 2)).unwrap();
        let parameter = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
        assert_eq!(
            center_support.parallel_derivative_scale_sign(&parameter, &CurveContext::STRICT).unwrap(),
            Classification::Decided(RealSign::Zero),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for clockwise in [false, true] {
                for signed_radius in [q(1, 8), q(-1, 8)] {
                    let Classification::Decided(Some(circle)) =
                        BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                            center_support.clone(), parameter.clone(), signed_radius.clone(),
                            clockwise, &policy,
                        ).unwrap()
                    else { panic!("a center-locus cusp retains a regular source frame"); };
                    let assert_point = |actual: Classification<CurvePoint2>, x: Real, y: Real| {
                        let Classification::Decided(actual) = actual else {
                            panic!("the cusp-centered circle point must remain exact");
                        };
                        assert_eq!(actual.same_point(&CurvePoint2::from(Point2::new(x, y)), &CurveContext::STRICT), Classification::Decided(true));
                    };
                    let midpoint_x = if clockwise { signed_radius.clone() } else { -signed_radius.clone() };
                    assert_point(circle.center_point_evidence(&CurveContext::STRICT).unwrap(), Real::zero(), q(1, 2));
                    assert_point(circle.start_point_evidence(&CurveContext::STRICT).unwrap(), Real::zero(), q(1, 2) + &signed_radius);
                    assert_point(circle.point_evidence_at(&q(1, 2), &CurveContext::STRICT).unwrap(), midpoint_x.clone(), q(1, 2));
                    assert_point(circle.end_point_evidence(&CurveContext::STRICT).unwrap(), Real::zero(), q(1, 2) - &signed_radius);
                    assert_point(circle.reversed().point_evidence_at(&q(1, 2), &CurveContext::STRICT).unwrap(), midpoint_x.clone(), q(1, 2));
                    assert_point(circle.complementary_half().point_evidence_at(&q(1, 2), &CurveContext::STRICT).unwrap(), -midpoint_x, q(1, 2));
                    let Classification::Decided(Some(offset)) = circle.offset_left(&q(1, 32), &CurveContext::STRICT).unwrap() else {
                        panic!("the selected circle keeps its frame through concentric offsets");
                    };
                    let radius_sign = if signed_radius.immediate_sign() == Some(RealSign::Negative) { -Real::one() } else { Real::one() };
                    let expected_radius = if clockwise { &signed_radius + radius_sign * q(1, 32) } else { &signed_radius - radius_sign * q(1, 32) };
                    assert_point(offset.start_point_evidence(&CurveContext::STRICT).unwrap(), Real::zero(), q(1, 2) + expected_radius);
                    assert_eq!(circle.data.frame.parallel_normal().unwrap().policy, CurveContext::STRICT);
                }
            }
            // An undefined source normal remains excluded. Zero center-locus
            // speed and zero source speed are different geometric premises.
            let stationary_source = QuadraticBezier2::new(
                Point2::from_values(0, 0), Point2::from_values(0, 0), Point2::from_values(1, 1),
            ).parallel_left(q(1, 2)).unwrap();
            assert!(matches!(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    stationary_source, parameter.clone(), q(1, 8), false, &policy,
                ).unwrap(), Classification::Uncertain(UncertaintyReason::Boundary),
            ));
        }
    }
