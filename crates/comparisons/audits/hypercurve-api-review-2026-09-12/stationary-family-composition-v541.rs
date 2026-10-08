#[cfg(test)]
mod stationary_family_composition_regression {
    use super::*;
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn joined_parallel_path(policy: &CurveContext, regular_ends: bool) -> CurvePath2 {
        // P(u)=(3u²/8,9u⁴/64). These reversed offsets join exactly at
        // (0,41/64), while radius 1/128 gives a continuous center family.
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let curves = [(q(41, 64), true), (q(5, 8), false)].map(|(distance, previous)| {
            let parallel = source.with_distance(distance);
            let point = |t: &CurveParameter2| {
                if matches!(t.as_bezier_parameter(), Some(BezierParameter2::Exact(value)) if value == &Real::zero()) {
                    return CurvePoint2::from(Point2::new(Real::zero(), parallel.distance().clone()));
                }
                analytic_parallel_point_evidence(
                    &parallel,
                    t,
                    CurveOperation2::Fillet,
                    CurveFamily2::AnalyticParallel,
                    policy,
                )
                .unwrap()
            };
            let range = if regular_ends {
                if previous {
                    CurveParameterRange2::new_validated(
                        Real::zero().into(),
                        (q(5, 9) + q(1, 10000)).sqrt().unwrap().into(),
                    )
                } else {
                    CurveParameterRange2::new_validated(
                        (q(5, 9) - q(1, 10000)).sqrt().unwrap().into(),
                        Real::one().into(),
                    )
                }
            } else {
                CurveParameterRange2::unit()
            };
            let start = point(range.start());
            let end = point(range.end());
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel),
                range,
                start,
                end,
            )
            .reversed()
            .into()
        });
        CurvePath2::try_new_with_policy(curves.into(), policy)
            .unwrap()
            .value
    }

    #[test]
    fn normalized_region_selects_and_reuses_a_stationary_fillet_family() {
        let parameter = CurveParameter2::from(q(5, 9).sqrt().unwrap());
        let join = CurvePoint2::from(Point2::new(Real::zero(), q(41, 64)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let path = joined_parallel_path(&policy, true);
            let mut curves = path.curves().to_vec();
            let Classification::Decided(closing) =
                crate::BezierAlgebraicChord2::try_new(path.end(), path.start(), &policy).unwrap()
            else {
                panic!("the exact closing chord must be certified");
            };
            curves.push(closing.into());
            let closed = CurvePath2::try_new_with_policy(curves, &policy)
                .unwrap()
                .value;
            let source = crate::CurveRegion2::try_from_boundary_paths(&[closed], &policy).unwrap();
            assert_eq!(source.certainty, crate::CurveCertainty::Certified);
            let source = source.value.regularized_region(&policy).unwrap();
            assert_eq!(source.certainty, crate::CurveCertainty::Certified);
            let source = source.value;
            let mut edited = Vec::new();
            let mut corners = 0;
            for (loop_index, boundary) in source.boundary_loops().iter().enumerate() {
                for (vertex, curve) in boundary.curves().iter().enumerate() {
                    if curve.start().coincides_with(&join, &policy).value
                        != Classification::Decided(true)
                    {
                        continue;
                    }
                    corners += 1;
                    let mut request = CurveFillet2::new(q(1, 128));
                    assert!(matches!(
                        source.fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy
                        ),
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ));
                    request.contacts[0] = Some(CurveFilletContact2::Parameter(parameter.clone()));
                    let selected = source
                        .fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    for region in selected.value.into_solutions() {
                        assert!(!region.is_empty());
                        let normalized = region.regularized_region(&policy).unwrap();
                        assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
                        let boolean = region.boolean_regions(&normalized.value, &policy).unwrap();
                        assert_eq!(boolean.certainty, crate::CurveCertainty::Certified);
                        assert!(boolean.value.xor().is_empty());
                        // Continue through the other corner and set operations
                        // with the selected output as their actual input.
                        let apex = CurvePoint2::from(Point2::new(
                            q(-175, 4992) + q(12, 1664),
                            q(4699, 7488) + q(5, 1664),
                        ));
                        let (arc_loop, arc_vertex) = region
                            .boundary_loops()
                            .iter()
                            .enumerate()
                            .find_map(|(loop_index, boundary)| {
                                boundary
                                    .curves()
                                    .iter()
                                    .position(|curve| {
                                        curve.start().coincides_with(&apex, &policy).value
                                            == Classification::Decided(true)
                                    })
                                    .map(|vertex| (loop_index, vertex))
                            })
                            .expect(
                                "the semicircle's two rational charts retain their common apex",
                            );
                        let chamfered = region
                            .chamfer_loop_vertex_by_setbacks(
                                arc_loop,
                                arc_vertex,
                                q(1, 4096),
                                q(1, 4096),
                                CurveCornerMode2::TrimOnly,
                                &policy,
                            )
                            .unwrap();
                        assert_eq!(chamfered.certainty, crate::CurveCertainty::Certified);
                        let CurveCornerSolutions2::Unique(chamfered) = chamfered.value else {
                            panic!("one exact circular-seam chamfer");
                        };
                        let offset = chamfered
                            .offset(q(1, 4096), &crate::OffsetCornerStyle2::Round, &policy)
                            .unwrap();
                        assert_eq!(offset.certainty, crate::CurveCertainty::Certified);
                        let vertices = [
                            Point2::new(q(-175, 4992), Real::zero()),
                            Point2::from_values(1, 0),
                            Point2::from_values(1, 1),
                            Point2::new(q(-175, 4992), Real::one()),
                        ];
                        let clip = CurvePath2::try_new(
                            (0..4)
                                .map(|i| {
                                    LineSeg2::try_new(
                                        vertices[i].clone(),
                                        vertices[(i + 1) % 4].clone(),
                                    )
                                    .unwrap()
                                    .into()
                                })
                                .collect(),
                        )
                        .unwrap();
                        let clip =
                            crate::CurveRegion2::try_from_boundary_paths(&[clip], &policy).unwrap();
                        assert_eq!(clip.certainty, crate::CurveCertainty::Certified);
                        let clipped = offset.value.boolean_regions(&clip.value, &policy).unwrap();
                        assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
                        assert!(!clipped.value.intersection().is_empty());
                        assert!(!clipped.value.difference().is_empty());
                        edited.push(region);
                    }
                }
            }
            assert_eq!(
                edited.len(),
                1,
                "one normalized boundary owns the selected fillet; corners={corners}"
            );
        }
    }
}
