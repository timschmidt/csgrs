    #[test]
    fn finite_point_inverse_replays_nonrepresented_center_cuts() {
        use crate::curve_intersection::CurveCircleOverlap2 as Overlap;
        let (center_x, quarter) = nonrepresented_center_rational_quarter();
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        let cutter = RationalBezier2::try_new(
            vec![
                Point2::new(center_x.clone(), q(3, 5)),
                Point2::new(&center_x + q(4, 5), q(3, 5)),
            ],
            vec![Real::one(); 2],
        ).unwrap();
        // The independent cut is C+(4/5,3/5), hence t=1/3 on
        // R(t)=C+((1-t^2)/(1+t^2),2t/(1+t^2)). A chord setback
        // of 1/4 rotates it clockwise with cos(delta)=31/32 and
        // sin(delta)=3sqrt(7)/32, giving the second exact parameter.
        let root7 = Real::from(7).sqrt().unwrap();
        let chamfer_parameter = ((Real::from(93) - Real::from(12) * &root7)
            / (Real::from(284) + Real::from(9) * &root7)).unwrap();
        let mut cases = 0;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = synthetic_independent_unit_cusp_semicircle_with_center_x(
                vec![Real::zero(), Real::one()], &policy,
            );
            assert!(circle.exact_center(&policy).unwrap().is_none());
            let Classification::Decided((
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps },
                Some(map),
            )) = circle.rational_intersections_with_parameter_map(
                &cutter, &CurveParameterRange2::unit(), &policy,
            ).unwrap() else { panic!("independent transverse endpoint") };
            assert!(overlaps.is_empty());
            let [contact] = contacts.as_slice() else { panic!("one first-quadrant contact") };
            let cut = map.contact_parameter(contact);
            let Classification::Decided(fragment) = BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle.clone(), BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                cut.clone(), false, &policy,
            ).unwrap() else { panic!("finite circle fragment") };
            let Classification::Decided(Some((chamfer_cut, _, false))) = fragment
                .endpoint_chord_setback_cut(false, &q(1, 4), false, &policy).unwrap()
            else { panic!("unique inward chamfer cut") };
            for shift in [0, 2, -2] {
                for reversed in [false, true] {
                    let source = if reversed { quarter.reversed() } else { quarter.clone() };
                    let Classification::Decided(source) = source.subcurve_between_affine_exact(
                        &Real::from(-shift), &Real::from(1-shift), &policy,
                    ).unwrap() else { panic!("exact target chart") };
                    let range = CurveParameterRange2::new_validated(
                        Real::from(shift).into(), Real::from(shift+1).into(),
                    );
                    let overlaps: Vec<_> = match circle.rational_intersections(&source, &range, &policy).unwrap() {
                        Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps }) => {
                            assert!(contacts.is_empty()); overlaps.into_iter().map(Overlap::Mapped).collect()
                        }
                        Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { contacts, overlaps }) => {
                            assert!(contacts.is_empty()); overlaps.into_iter().map(Overlap::Selected).collect()
                        }
                        other => panic!("finite nonrepresented-center overlap: {other:?}"),
                    };
                    let [overlap] = overlaps.as_slice() else { panic!("one circle overlap") };
                    for (index, (cut, expected)) in [(&cut, q(1, 3)), (&chamfer_cut, chamfer_parameter.clone())].into_iter().enumerate() {
                        let expected = Real::from(shift) + if reversed { Real::one() - expected } else { expected };
                        let outcome = crate::policy::resolve_certified_operation(&policy, |attempt| {
                            let cut = CurveParameter2::from_algebraic_cusp(cut.clone());
                            match overlap {
                                Overlap::Mapped(map) => map.map_parameter(&cut, true, attempt),
                                Overlap::Selected(map) => map.map_parameter(&cut, true, attempt),
                                Overlap::Pair(_) => unreachable!(),
                            }
                        }).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let Classification::Decided(Some(parameter)) = outcome.value else {
                            panic!("finite point inverse: shift={shift}, reversed={reversed}, cut={index}, policy={policy:?}: {:?}", outcome.value)
                        };
                        assert_eq!(parameter.same_value(&expected.into(), &policy).unwrap(), Classification::Decided(true));
                        cases += 1;
                        println!("passed shift={shift} reversed={reversed} cut={index} policy={policy:?}");
                    }
                }
            }
        }
        assert_eq!(cases, 24);
    }
