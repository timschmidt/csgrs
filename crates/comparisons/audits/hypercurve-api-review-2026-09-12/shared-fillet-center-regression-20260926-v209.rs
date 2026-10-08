    #[test]
    fn distinct_parallel_sources_keep_a_shared_fillet_center_family() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(3t/8,9t^2/64). The reversed offsets 41/64 and 5/8
        // join at (0,41/64). Their common clockwise radius-1/128 center
        // support is the offset 81/128. At t=5/9 the two original
        // derivative scales are -17/2197 and 37/2197, respectively.
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(3, 16), Real::zero()),
            Point2::new(q(3, 8), q(9, 64)),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let original = [source.with_distance(q(41, 64)), source.with_distance(q(5, 8))];
        let radius = q(1, 128);
        let center_distance = q(81, 128);
        let parameter = CurveParameter2::from(q(5, 9));
        let range = CurveParameterRange2::new_validated(
            CurveParameter2::from(q(5, 9) - q(1, 10_000)),
            CurveParameter2::from(q(5, 9) + q(1, 10_000)),
        );
        let contacts = [
            Point2::new(q(-95, 2496), q(4753, 7488)),
            Point2::new(q(-5, 156), q(4645, 7488)),
        ];
        let center = Point2::new(q(-175, 4992), q(4699, 7488));
        for point in &contacts {
            let dx = point.x() - center.x();
            let dy = point.y() - center.y();
            assert_eq!(&dx * &dx + &dy * &dy, &radius * &radius);
        }
        assert_ne!(contacts[0], contacts[1]);
        let domains = [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point_at = |parallel: &BezierParallel2, parameter: &CurveParameter2| {
                analytic_parallel_point_evidence(
                    parallel, parameter, CurveOperation2::Fillet,
                    CurveFamily2::AnalyticParallel, &policy,
                ).unwrap()
            };
            for (parallel, expected) in original.iter().zip(&contacts) {
                assert_eq!(point_at(parallel, &parameter).same_point(
                    &CurvePoint2::from(expected.clone()), &policy,
                ), Classification::Decided(true));
            }
            assert_eq!(point_at(&source.with_distance(center_distance.clone()), &parameter)
                .same_point(&CurvePoint2::from(center.clone()), &policy),
                Classification::Decided(true));
            let selected = original.each_ref().map(|parallel| {
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                    range.clone(), point_at(parallel, range.start()), point_at(parallel, range.end()),
                ).reversed()
            });
            for reversed in [false, true] {
                let curves = if reversed {
                    [selected[1].reversed(), selected[0].reversed()]
                } else {
                    selected.clone()
                };
                let prepared = curves.each_ref().map(|curve| {
                    PreparedFilletCarrier2::new(
                        ExactCornerCarrier2::SelectedFiber(curve),
                        CurveFamily2::AnalyticParallel, domains[0], &policy,
                    ).unwrap()
                });
                let signed_radius = if reversed { radius.clone() } else { -radius.clone() };
                let previous_offsets = prepared[0].offsets(
                    &signed_radius, CurveFamily2::AnalyticParallel, &policy,
                ).unwrap();
                let next_offsets = prepared[1].offsets(
                    &signed_radius, CurveFamily2::AnalyticParallel, &policy,
                ).unwrap();
                let is_shared = |offset: &&FilletOffsetCarrier2<'_, '_>| matches!(
                    offset, FilletOffsetCarrier2::Parallel { support, .. }
                    if support.distance() == &center_distance
                );
                let previous = previous_offsets.iter().flatten().find(is_shared)
                    .expect("the previous normal sheet reaches the rational center");
                let next = next_offsets.iter().flatten().find(is_shared)
                    .expect("the next normal sheet reaches the same rational center");
                let accepted = |parameter: &CurveParameter2| {
                    [previous, next].into_iter().zip(&prepared).map(|(offset, prepared)| {
                        prepared.accepts_offset_contact(
                            offset, Some(parameter), &signed_radius,
                            CurveFamily2::AnalyticParallel, &policy,
                        ).unwrap()
                    }).all(|accepted| accepted)
                };
                assert!(accepted(&parameter));
                // Coincident centers alone do not establish original tangent
                // orientation. Outside the opposite-sign cusp band these
                // same center supports are incompatible normal sheets.
                assert!(!accepted(&CurveParameter2::from(Real::zero())));
                assert!(!accepted(&CurveParameter2::from(Real::one())));
                let centers = fillet_offset_centers(
                    previous, next, domains, CurveFamily2::AnalyticParallel,
                    CurveFamily2::AnalyticParallel, &policy,
                ).expect("the exact common center support must classify");
                assert!(centers.coincident,
                    "different original offsets have a noncollapsed diagonal center family");
            }
        }
    }
