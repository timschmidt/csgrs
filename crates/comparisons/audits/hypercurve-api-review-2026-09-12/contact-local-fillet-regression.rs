    #[test]
    fn fillet_center_contacts_keep_source_orientation_across_support_cusps() {
        let p = Point2::from_values;
        let half = (Real::one() / Real::from(2)).unwrap();
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let curve = QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0));
        let authored = Curve2::from(curve.clone());
        let line = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let prepared = PreparedFilletCarrier2::new(
                ExactCornerCarrier2::Line(&line), CurveFamily2::Line, &policy,
            ).unwrap();
            let previous = prepared.offset(&half, CurveFamily2::Line, &policy).unwrap();
            let parallel = curve.parallel_left(Real::zero()).unwrap();
            let Classification::Decided(retained) = crate::BezierParallelFragment2::try_new(
                parallel.clone(),
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ), &policy,
            ).unwrap() else { panic!("the zero-distance cap is regular"); };
            let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                CurveParameterRange2::unit(), p(4, 0).into(), p(2, 0).into(),
            );
            let reversed_retained = retained.reversed();
            let reversed_selected = selected.reversed();
            for (source, reversed) in [
                (FilletParallelSource2::Direct(ExactCornerBezier2::Direct(&authored)), false),
                (FilletParallelSource2::Retained(&retained), false),
                (FilletParallelSource2::Selected(&selected), false),
                (FilletParallelSource2::Retained(&reversed_retained), true),
                (FilletParallelSource2::Selected(&reversed_selected), true),
            ] {
                let next = FilletOffsetCarrier2::Parallel {
                    source, support: parallel.with_distance(half.clone()),
                };
                let centers = fillet_offset_centers(
                    &previous, &next,
                    [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2],
                    CurveFamily2::Line, CurveFamily2::QuadraticBezier, &policy,
                ).unwrap();
                assert_eq!(centers.iter().count(), 2);
                for center in centers.iter() {
                    let parameter = center.next_parameter.as_ref().unwrap();
                    let Classification::Decided(side) = parameter.polynomial_sign(
                        &[-half.clone(), Real::one()], &policy,
                    ).unwrap() else { panic!("each contact lies on one side of the cap"); };
                    let boundary = if side == RealSign::Negative {
                        quarter.clone()
                    } else { Real::one() - &quarter };
                    assert_eq!(parameter.polynomial_sign(&[-boundary, Real::one()], &policy).unwrap(),
                               Classification::Decided(side));
                    // P'=(-2,8-16t): its dot with the positive x-axis is
                    // negative, and P' x (1,0) has the sign of t-1/2.
                    // At these outer contacts the offset scale is positive;
                    // at the unrelated midpoint it is negative (radius 1/4).
                    let evidence = center.retained_anchor_evidence.as_ref().unwrap();
                    assert_eq!(evidence.dot, Some(if reversed { RealSign::Positive } else { RealSign::Negative }));
                    assert_eq!(evidence.cross, Some(if reversed { reverse_fillet_sign(side) } else { side }));
                    assert_eq!(evidence.source_direction, Some(if reversed { RealSign::Negative } else { RealSign::Positive }));
                }
            }
        }
    }
