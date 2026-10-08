use hypercurve::*;

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn decided<T>(result: Classification<T>) -> T {
    match result {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("{reason:?}"),
    }
}
fn certified<T>(result: CurveOutcome<T>) -> T {
    assert_eq!(result.certainty, CurveCertainty::Certified);
    result.value
}
fn chord(start: Point2, end: Point2, policy: &CurveContext) -> BezierSplitFragment2 {
    BezierSplitFragment2::AlgebraicChord(decided(
        BezierAlgebraicChord2::try_new(start.into(), end.into(), policy).unwrap(),
    ))
}
fn inspect(
    label: &str,
    region: &CurveRegion2,
    cap: bool,
    policy: &CurveContext,
    checks: &mut usize,
) {
    let samples = if cap {
        vec![
            (p(2, 0), RegionPointLocation::Inside),
            (p(2, -1), RegionPointLocation::Outside),
            (p(2, 1), RegionPointLocation::Outside),
            (
                Point2::new(2.into(), q(-1, 4)),
                RegionPointLocation::Boundary,
            ),
        ]
    } else {
        vec![
            (p(3, 2), RegionPointLocation::Inside),
            (p(3, 0), RegionPointLocation::Outside),
            (p(3, 4), RegionPointLocation::Outside),
            (p(3, 1), RegionPointLocation::Boundary),
        ]
    };
    for (point, expected) in samples {
        let result = region.classify_point(&point, policy);
        println!("{label}: point={point:?} expected={expected:?} result={result:?}");
        assert_eq!(decided(certified(result.unwrap())), expected);
        *checks += 1;
    }
}
fn main() {
    let mut checks = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for cap in [false, true] {
            for finite in [false, true] {
                for reverse in [false, true] {
                    let label =
                        format!("cap={cap} finite={finite} reverse={reverse} policy={policy:?}");
                    println!("start {label}");
                    let (source, distance, start, end) = match (cap, finite) {
                        // P(t)=(t,(t-2)^2), [1,3], versus P(2s+1), [0,1].
                        (true, true) => (
                            QuadraticBezier2::new(p(0, 4), Point2::new(q(1, 2), 2.into()), p(1, 1)),
                            q(-1, 4),
                            1,
                            3,
                        ),
                        (true, false) => (
                            QuadraticBezier2::new(p(1, 1), p(2, -1), p(3, 1)),
                            q(-1, 4),
                            0,
                            1,
                        ),
                        // P(t)=(t,0), [2,4], versus P(2s+2), [0,1].
                        (false, true) => (
                            QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), 0.into()), p(1, 0)),
                            Real::one(),
                            2,
                            4,
                        ),
                        (false, false) => (
                            QuadraticBezier2::new(p(2, 0), p(3, 0), p(4, 0)),
                            Real::one(),
                            0,
                            1,
                        ),
                    };
                    let parallel = source.parallel_left(distance).unwrap();
                    let start_point =
                        decided(parallel.point_at(&Real::from(start), &policy).unwrap());
                    let end_point = decided(parallel.point_at(&Real::from(end), &policy).unwrap());
                    let range = decided(
                        BezierParameterRange2::try_new(
                            BezierParameter2::Exact(start.into()),
                            BezierParameter2::Exact(end.into()),
                            &policy,
                        )
                        .unwrap(),
                    );
                    let fragment = decided(
                        BezierParallelFragment2::try_new(parallel, range, &policy).unwrap(),
                    );
                    let mut fragments = vec![BezierSplitFragment2::AnalyticParallel(fragment)];
                    if cap {
                        fragments.push(chord(end_point, start_point, &policy));
                    } else {
                        fragments.extend([
                            chord(end_point, p(4, 3), &policy),
                            chord(p(4, 3), p(2, 3), &policy),
                            chord(p(2, 3), start_point, &policy),
                        ]);
                    }
                    if reverse {
                        fragments = fragments
                            .into_iter()
                            .rev()
                            .map(|fragment| fragment.reversed().unwrap())
                            .collect();
                    }
                    let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
                    let region = CurveRegion2::try_new_with_loop_topology(
                        vec![boundary],
                        vec![CurveRegionLoopRole::Material],
                        vec![FillRule::NonZero],
                        vec![if reverse {
                            CurveBoundaryInteriorSide2::Right
                        } else {
                            CurveBoundaryInteriorSide2::Left
                        }],
                    )
                    .unwrap();
                    inspect(
                        &format!("{label} admitted"),
                        &region,
                        cap,
                        &policy,
                        &mut checks,
                    );
                    let paths = decided(certified(region.boundary_paths(&policy).unwrap()));
                    assert_eq!(paths.len(), 1);
                    let replay = certified(
                        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                            &paths,
                            &[CurveRegionLoopRole::Material],
                            &[FillRule::NonZero],
                            &policy,
                        )
                        .unwrap(),
                    );
                    inspect(
                        &format!("{label} replay"),
                        &replay,
                        cap,
                        &policy,
                        &mut checks,
                    );
                    // Authored region re-entry currently loses side evidence for a
                    // nonintegrable cap; retain this diagnostic independently.
                    let raw_sides = replay.filled_side_is_left(&policy).unwrap();
                    println!("{label} raw filled sides={raw_sides:?}");
                    let normalized = certified(replay.regularized_region(&policy).unwrap());
                    inspect(
                        &format!("{label} normalized"),
                        &normalized,
                        cap,
                        &policy,
                        &mut checks,
                    );
                    assert_eq!(
                        decided(certified(normalized.filled_side_is_left(&policy).unwrap())),
                        vec![true]
                    );
                    checks += 1;
                }
            }
        }
    }
    println!("checks={checks}");
    assert_eq!(checks, 208);
}
