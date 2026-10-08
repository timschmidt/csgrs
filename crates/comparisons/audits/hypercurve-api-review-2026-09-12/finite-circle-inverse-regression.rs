    #[test]
    fn finite_circle_tangent_inverse_replays_independent_cuts() {
        use crate::curve_intersection::CurveCircleOverlap2 as Overlap;
        let half = (Real::one() / Real::from(2)).unwrap();
        let quarter = RationalBezier2::from(
            RationalQuadraticBezier2::try_new(
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
                Point2::from_values(0, 1),
                Real::one(), Real::one(), Real::from(2),
            ).unwrap(),
        );
        let cutter = RationalBezier2::try_new(
            vec![Point2::new(Real::zero(), half.clone()), Point2::new(Real::one(), half)],
            vec![Real::one(); 2],
        ).unwrap();
        // R(t)=((1-t^2)/(1+t^2), 2t/(1+t^2)) meets y=1/2
        // once in the first quadrant, at t=2-sqrt(3). The cut is authored
        // by an independent line/circle contact, never by the target map.
        let expected = Real::from(2) - Real::from(3).sqrt().unwrap();
        let mut failures = 0;
        let mut cases = 0;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = synthetic_reducible_cusp_semicircle((3, 4), ((2, 3), (4, 5)), &policy);
            let Classification::Decided((
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps },
                Some(map),
            )) = circle.rational_intersections_with_parameter_map(
                &cutter, &CurveParameterRange2::unit(), &policy,
            ).unwrap() else { panic!("independent transverse circle cut") };
            assert!(overlaps.is_empty());
            let [contact] = contacts.as_slice() else { panic!("one first-quadrant cut") };
            let cut = CurveParameter2::from_algebraic_cusp(map.contact_parameter(contact));
            for shift in [0, 2, -2] {
                for reversed in [false, true] {
                    for analytic in [false, true] {
                        let shift = Real::from(shift);
                        let source = if reversed { quarter.reversed() } else { quarter.clone() };
                        let Classification::Decided(source) = source.subcurve_between_affine_exact(
                            &(-shift.clone()), &(Real::one() - &shift), &policy,
                        ).unwrap() else { panic!("exterior circle chart") };
                        let range = if reversed {
                            CurveParameterRange2::new_validated((&shift + Real::one()).into(), shift.clone().into())
                        } else {
                            CurveParameterRange2::new_validated(shift.clone().into(), (&shift + Real::one()).into())
                        };
                        let overlaps: Vec<_> = if analytic {
                            let scale = Similarity2::try_from_real_affine(
                                Real::from(2), Real::zero(), Real::zero(), Real::from(2), Real::zero(), Real::zero(),
                            ).unwrap();
                            let parallel = source.transform_similarity(&scale)
                                .parallel_left(if reversed { -Real::one() } else { Real::one() }).unwrap();
                            assert!(parallel.data.certified_ph_offset.set(None).is_ok());
                            match circle.parallel_intersections(&parallel, &range, None, &policy).unwrap() {
                                Classification::Decided(BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps }) => {
                                    assert!(contacts.is_empty());
                                    overlaps.into_iter().map(Overlap::Mapped).collect()
                                }
                                other => panic!("finite analytic circle overlap: {other:?}"),
                            }
                        } else {
                            match circle.rational_intersections(&source, &range, &policy).unwrap() {
                                Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps }) => {
                                    assert!(contacts.is_empty());
                                    overlaps.into_iter().map(Overlap::Mapped).collect()
                                }
                                Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { contacts, overlaps }) => {
                                    assert!(contacts.is_empty());
                                    overlaps.into_iter().map(Overlap::Selected).collect()
                                }
                                other => panic!("finite rational circle overlap: {other:?}"),
                            }
                        };
                        let [overlap] = overlaps.as_slice() else { panic!("one quarter-circle overlap") };
                        let outcome = crate::policy::resolve_certified_operation(&policy, |attempt| {
                            match overlap {
                                Overlap::Mapped(map) => map.map_parameter(&cut, true, attempt),
                                Overlap::Selected(map) => map.map_parameter(&cut, true, attempt),
                                Overlap::Pair(_) => unreachable!(),
                            }
                        }).unwrap();
                        println!("shift={shift:?} reversed={reversed} analytic={analytic} policy={policy:?}: {:?}", outcome.value);
                        cases += 1;
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let Classification::Decided(Some(parameter)) = outcome.value else {
                            failures += 1;
                            continue;
                        };
                        let expected = &shift + if reversed { Real::one() - &expected } else { expected.clone() };
                        assert_eq!(parameter.same_value(&expected.into(), &policy).unwrap(), Classification::Decided(true));
                    }
                }
            }
        }
        assert_eq!(cases, 24);
        assert_eq!(failures, 0, "finite inverses lost independent cuts");
    }
