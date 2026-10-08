    #[test]
    fn native_rational_pair_evidence_clips_both_retained_domains() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for distance in [0_i64, 1] {
                let line = QuadraticBezier2::new(p(0,-distance), Point2::new(q(1,2),Real::from(-distance)), p(1,-distance))
                    .parallel_left(Real::from(distance)).unwrap();
                // The offset line is y=0. Q(v)=(v,(v-1/4)(v-3/4))
                // has exactly the two native contacts (u,v)=(1/4,1/4),(3/4,3/4).
                let curve = QuadraticBezier2::new(Point2::new(Real::zero(),q(3,16)),Point2::new(q(1,2),q(-5,16)),Point2::new(Real::one(),q(3,16)))
                    .parallel_left(Real::zero()).unwrap();
                let range = |a:Real,b:Real| CurveParameterRange2::new_validated(a.into(),b.into());
                for (first,second,expected) in [
                    (range(Real::zero(),q(1,2)),CurveParameterRange2::unit(),Some(q(1,4))),
                    (CurveParameterRange2::unit(),range(q(1,2),Real::one()),Some(q(3,4))),
                    (range(Real::zero(),q(1,2)),range(q(1,2),Real::one()),None),
                    (range(Real::one(),q(1,2)),range(Real::one(),Real::zero()),Some(q(3,4))),
                ] {
                    for swapped in [false,true] {
                        let (a,b,ar,br)=if swapped {(&curve,&line,&second,&first)}else{(&line,&curve,&first,&second)};
                        let result=decided(a.parallel_intersections_on_regular_ranges(b,ar,br,&policy).unwrap());
                        assert!(result.is_complete());
                        assert_eq!(result.contacts().len(),usize::from(expected.is_some()));
                        assert!(result.overlaps().is_empty());assert!(result.parameter_components().is_empty());
                        if let Some(value)=&expected {
                            let contact=&result.contacts()[0];
                            for parameter in [contact.first_parameter(),contact.second_parameter()] {
                                assert_eq!(parameter.polynomial_sign(&[-value,Real::one()],&policy).unwrap(),Classification::Decided(RealSign::Zero));
                            }
                        }
                    }
                }
            }
        }
    }
