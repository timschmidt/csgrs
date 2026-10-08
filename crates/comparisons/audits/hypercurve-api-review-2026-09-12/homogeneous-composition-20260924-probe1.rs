use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn homogeneous_boundary_closes_through_boolean_corners_and_offset() {
    let clock = std::time::Instant::now();
    let mark = |label: &str| eprintln!("stage={} elapsed_ms={}", label, clock.elapsed().as_millis());
    mark("begin");
    let admit = |path, policy: &CurveContext| {
        let admitted = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[hypercurve::CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            policy,
        )
        .unwrap();
        assert_eq!(admitted.certainty, CurveCertainty::Certified);
        admitted.into_value()
    };
    let check = |region: &CurveRegion2, policy: &CurveContext| {
        assert_eq!(region.boundary_loops().len(), 1);
        for (point, expected) in [
            (Point2::new(q(1, 4), q(1, 4)), RegionPointLocation::Inside),
            (Point2::new(q(-1, 2), q(1, 2)), RegionPointLocation::Outside),
        ] {
            let result = region.classify_point(&point, policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value, Classification::Decided(expected));
        }
    };
    for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512].into_iter().enumerate() {
        eprintln!("policy={policy_index}");
        // The middle homogeneous control is at infinity; the exact upper
        // semicircle and its complete authored denominator remain finite.
        let Classification::Decided(curve) = RationalBezier2::from_homogeneous_controls(
            vec![
                HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
                HomogeneousControl2::new(Real::zero(), Real::one(), Real::zero()),
                HomogeneousControl2::new(-Real::one(), Real::zero(), Real::one()),
            ],
            &policy,
        )
        .unwrap() else {
            panic!("the homogeneous semicircle must construct");
        };
        assert!(curve.affine_control_points().is_none());
        // Independently authored elevated controls must recover their small
        // exact source even though that source has no affine control net.
        let Classification::Decided(elevated) = RationalBezier2::from_homogeneous_controls(
            curve
                .elevated_to_degree(12)
                .unwrap()
                .homogeneous_controls()
                .to_vec(),
            &policy,
        )
        .unwrap() else {
            panic!("the elevated homogeneous semicircle must construct");
        };
        let nurbs = hypercurve::NurbsCurve2::from_homogeneous_controls(
            2,
            curve.homogeneous_controls().to_vec(),
            vec![
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::one(),
                Real::one(),
            ],
            hypercurve::SplinePeriodicity2::NonPeriodic,
            &policy,
        )
        .unwrap()
        .into_value();
        mark("construct-families");
        for (source_index, curve) in [
            Curve2::from(curve),
            Curve2::from(elevated),
            Curve2::from(nurbs.elevated_to_degree(12, &policy).unwrap().into_value()),
            Curve2::from(nurbs),
        ].into_iter().enumerate() {
            eprintln!("source={source_index}");
            mark("admit-material:start");
            let material = admit(
                CurvePath2::try_new(vec![
                    curve,
                    Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap()),
                ])
                .unwrap(),
                &policy,
            );
            mark("admit-material:done");
            let rectangle = admit(
                CurvePath2::try_new(
                    [p(0, -1), p(2, -1), p(2, 2), p(0, 2), p(0, -1)]
                        .windows(2)
                        .map(|points| {
                            Curve2::from(
                                LineSeg2::try_new(points[0].clone(), points[1].clone()).unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap(),
                &policy,
            );
            mark("admit-rectangle:done");
            mark("boolean:start");
            let results = material.boolean_regions(&rectangle, &policy).unwrap();
            assert_eq!(results.certainty, CurveCertainty::Certified);
            mark("boolean:done");
            let clipped = results.value.intersection();
            mark("classify-clipped:start");
            check(clipped, &policy);
            mark("classify-clipped:done");
            for fillet in [false, true] {
                eprintln!("fillet={fillet}");
                mark("corner:start");
                let solutions = if fillet {
                    clipped.fillet_loop_vertex_by_radius(
                        0,
                        1,
                        q(1, 8),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                } else {
                    clipped.chamfer_loop_vertex_by_setbacks(
                        0,
                        1,
                        q(1, 8),
                        q(1, 8),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                }
                .unwrap();
                mark("corner:done");
                assert_eq!(solutions.certainty, CurveCertainty::Certified);
                let regions = match solutions.value {
                    CurveCornerSolutions2::Unique(region) => vec![region],
                    CurveCornerSolutions2::Multiple(regions) => regions,
                    CurveCornerSolutions2::NoSolution(reason) => {
                        panic!("the clipped corner must admit an edit: {reason:?}")
                    }
                };
                assert!(!regions.is_empty());
                eprintln!("corner_candidates={}",regions.len());
                for (candidate_index, edited) in regions.into_iter().enumerate() {
                    eprintln!("candidate={candidate_index}");
                    mark("classify-edited:start");
                    check(&edited, &policy);
                    mark("classify-edited:done");
                    mark("offset:start");
                    let displaced = edited
                        .offset(q(1, 32), &OffsetCornerStyle2::Round, &policy)
                        .unwrap();
                    mark("offset:done");
                    assert_eq!(displaced.certainty, CurveCertainty::Certified);
                    mark("replay-boolean:start");
                    let replay = displaced
                        .value
                        .boolean_regions(&rectangle, &policy)
                        .unwrap();
                    mark("replay-boolean:done");
                    assert_eq!(replay.certainty, CurveCertainty::Certified);
                    mark("classify-replay:start");
                    check(replay.value.intersection(), &policy);
                    mark("classify-replay:done");
                }
            }
        }
    }
}

fn main(){homogeneous_boundary_closes_through_boolean_corners_and_offset();}
