    fn retained_frame_test_parameters(value: Real, policy: &CurveContext) -> [CurveParameter2; 3] {
        let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![-q(1,2),Real::zero(),Real::one()],policy).unwrap());
        let interval=decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),policy).unwrap());
        let alpha=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,policy).unwrap());
        let selected=BezierAlgebraicSelectedFiberAuthority2::exact_parameter(alpha,value.clone(),policy);
        let one=DenseTensorPolynomial::try_new(vec![],vec![Real::one()]).unwrap();
        let field=BezierRecursiveQuadraticField2::base(vec![],one.clone(),one).unwrap();
        let recursive=decided(BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
            BezierRecursiveQuadraticProjectiveScalar2 {
                numerator:field.constant(value.clone()).unwrap(),
                denominator:field.constant(Real::one()).unwrap(),
            },Some((Real::zero(),Real::one())),policy).unwrap());
        [value.into(),CurveParameter2::from_selected_fiber(selected),CurveParameter2::from_recursive_projective(recursive)]
    }

    #[test]
    fn regular_source_frames_accept_every_retained_parameter_authority() {
        let source=CubicBezier2::new(p(0,0),p(0,0),Point2::new(q(1,3),Real::zero()),p(1,1));
        let anchor=source.parallel_left(Real::one()).unwrap();
        let parallel=anchor.with_distance(Real::from(2));
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
            for parameter in retained_frame_test_parameters(Real::zero(),&policy) {
                for (range,normal) in [
                    (CurveParameterRange2::unit(),1_i64),
                    (CurveParameterRange2::new_validated((-Real::one()).into(),Real::zero().into()),-1),
                ] {
                    for direction in [RealSign::Positive,RealSign::Negative] {
                        let (point,tangent)=decided(parallel.regular_source_point_and_tangent_support(&anchor,&parameter,&range,direction,&policy).unwrap());
                        let dx=normal*if direction==RealSign::Positive {1} else {-1};
                        for (actual,expected) in [(&point,p(0,2*normal)),(tangent.start(),p(0,normal)),(tangent.end(),p(dx,normal))] {
                            assert_eq!(actual.same_point(&expected.into(),&policy),Classification::Decided(true));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn regular_source_frames_retain_unprojectable_selected_parameters() {
        let source=CubicBezier2::new(p(0,0),p(0,0),Point2::new(q(1,3),Real::zero()),p(1,1));
        let anchor=source.parallel_left(Real::one()).unwrap();
        let parallel=anchor.with_distance(Real::from(2));
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
            let selected=degree_nine_selected_fiber_parameter_for_test(q(1,2),32768,&policy);
            assert!(selected.data.representations.bezier.get().is_none());
            let parameter=CurveParameter2::from_selected_fiber(selected.clone());
            let (point,tangent)=decided(parallel.regular_source_point_and_tangent_support(&anchor,&parameter,&CurveParameterRange2::unit(),RealSign::Positive,&policy).unwrap());
            // At positive t, the raw tangent t(2,3t) and cancelled field
            // (2,3t) define the same unit normal and tangent. These independent
            // procedural expressions must compare without a global eliminant.
            for (actual,support,displacement) in [(&point,&parallel,Real::zero()),(tangent.start(),&anchor,Real::zero()),(tangent.end(),&anchor,Real::one())] {
                let expected=BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(support.clone(),&parameter,displacement,&policy).unwrap();
                assert_eq!(actual.same_point(&CurvePoint2::from(expected),&policy),Classification::Decided(true));
            }
            assert!(selected.data.representations.bezier.get().is_none());
        }
    }

    #[test]
    fn regular_source_frames_reject_poles_in_every_parameter_authority() {
        let source=RationalBezier2::try_new(vec![p(0,0),p(1,0)],vec![-Real::one(),Real::one()]).unwrap();
        let parallel=source.parallel_left(Real::one()).unwrap();
        let range=CurveParameterRange2::new_validated(q(1,2).into(),Real::one().into());
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
            for parameter in retained_frame_test_parameters(q(1,2),&policy) {
                assert!(matches!(parallel.regular_source_point_and_tangent_support(&parallel,&parameter,&range,RealSign::Positive,&policy).unwrap(),Classification::Uncertain(UncertaintyReason::Boundary)));
            }
        }
    }
