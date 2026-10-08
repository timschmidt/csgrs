    #[test]
    fn probe_constructed_circle_endpoint_retains_its_known_parameter() {
        let policy = CurveContext::STRICT;
        let fraction = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        let parameters = [
            fraction(1, 3),
            fraction(1, 2),
            fraction(1, 3).sqrt().unwrap(),
            fraction(2, 3).sqrt().unwrap(),
        ];
        let centers = [
            Point2::from_values(0, 0),
            Point2::new(fraction(1, 2).sqrt().unwrap(), fraction(1, 3).sqrt().unwrap()),
        ];
        let mut unresolved = 0;
        for (center_index, center) in centers.into_iter().enumerate() {
            let arc = CircularArc2::try_from_center(
                Point2::new(center.x() + Real::from(2), center.y().clone()),
                Point2::new(center.x().clone(), center.y() + Real::from(2)),
                center.clone(),
                false,
            ).unwrap();
            let Classification::Decided(decomposition) = arc
                .rational_bezier_decomposition_with_policy(&policy).unwrap()
            else { panic!("the exact quarter circle must decompose"); };
            let [span] = decomposition.spans() else { panic!("one quarter-circle span"); };
            let source = RationalBezier2::from(span.curve().clone());
            for (parameter_index, parameter) in parameters.iter().enumerate() {
                let Classification::Decided(point) = source.point_at_affine_classified(parameter, &policy)
                else { panic!("the known finite source point must evaluate"); };
                let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(
                    center.clone().into(), point.into(), &policy,
                ).unwrap() else { panic!("the radial chord must construct"); };
                let result = chord.rational_intersections(&source, &CurveParameterRange2::unit(), None, &policy).unwrap();
                match result {
                    Classification::Decided(BezierAlgebraicChordRationalIntersections2::Contacts(contacts)) => {
                        assert_eq!(contacts.len(), 1, "a finite radial chord has one circle endpoint");
                        let contact = &contacts[0];
                        let order = contact.other_parameter().cmp_by_refinement(
                            &CurveParameter2::from(BezierParameter2::Exact(parameter.clone())), &policy,
                        ).unwrap();
                        let endpoint = contact.chord_parameter().is_endpoint_of(&chord, false);
                        let valid = endpoint && order == Classification::Decided(std::cmp::Ordering::Equal)
                            && contact.tangent_cross_sign() == RealSign::Positive;
                        eprintln!("RADIAL_ENDPOINT center={center_index} parameter={parameter_index} contacts={} endpoint={endpoint} parameter_order={order:?} tangent={:?} valid={valid}", contacts.len(), contact.tangent_cross_sign());
                        unresolved += usize::from(!valid);
                    }
                    Classification::Uncertain(reason) => {
                        eprintln!("RADIAL_ENDPOINT center={center_index} parameter={parameter_index} reason={reason:?}");
                        unresolved += 1;
                    }
                    Classification::Decided(other) => panic!("unexpected radial contact kind {:?}", std::mem::discriminant(&other)),
                }
            }
        }
        assert_eq!(unresolved, 0, "all constructed radial endpoint contacts must replay exactly");
    }

