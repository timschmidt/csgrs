    #[test]
    fn recursive_incidence_accepts_cusp_contact_and_derived_points() {
        let half = (Real::one() / Real::from(2)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let tangent = CurvePoint2::from(strict_tangent_chord_query_point());
            let (_, quarter, overlap) = selected_fiber_rational_quarter_overlap(&policy);
            let derived = overlap.map.contact(
                overlap.other_end.clone(), BezierAlgebraicCuspSemicircleContactLocation2::Interior, RealSign::Zero,
            ).point_evidence();
            assert!(matches!(derived, CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))));
            for (point, expected) in [(tangent, Point2::from_values(1, 0)), (derived, quarter.end().clone())] {
                assert!(point.coordinates().is_none());
                let same = point.coincides_with(&CurvePoint2::from(expected.clone()), &policy);
                assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                assert_eq!(same.value, Classification::Decided(true));
                let source = QuadraticBezier2::new(
                    expected.translated(-Real::one(), -Real::one()),
                    expected.translated(Real::zero(), -Real::one()),
                    expected.translated(Real::one(), -Real::one()),
                ).parallel_left(Real::one()).unwrap();
                for distance in [-1, 1] {
                    let mut parameters = Vec::new();
                    assert_eq!(source.with_distance(Real::from(distance)).visit_point_incidence_evidence(
                        &point, &CurveParameterRange2::unit(), None, false, &policy,
                        &mut |parameter| { parameters.push(parameter.unwrap().clone()); ControlFlow::Continue(()) },
                    ).unwrap(), Classification::Decided(ControlFlow::Continue(())));
                    assert_eq!(parameters.len(), usize::from(distance > 0));
                    for parameter in parameters {
                        assert_eq!(parameter.same_value(&half.clone().into(), &policy).unwrap(), Classification::Decided(true));
                    }
                }
            }
        }
    }
