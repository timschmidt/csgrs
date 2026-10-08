use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn exact<T>(x: Classification<T>) -> T {
    match x {
        Classification::Decided(v) => v,
        Classification::Uncertain(e) => panic!("{e:?}"),
    }
}
fn certified<T>(x: CurveOutcome<T>) -> T {
    assert_eq!(x.certainty, CurveCertainty::Certified);
    x.value
}
fn cap(finite: bool, policy: &CurveContext) -> CurveRegion2 {
    let (source, start, end) = if finite {
        (
            QuadraticBezier2::new(p(0, 4), Point2::new(q(1, 2), 2.into()), p(1, 1)),
            1,
            3,
        )
    } else {
        (QuadraticBezier2::new(p(1, 1), p(2, -1), p(3, 1)), 0, 1)
    };
    let parallel = source.parallel_left(q(-1, 4)).unwrap();
    let a = exact(parallel.point_at(&start.into(), policy).unwrap());
    let b = exact(parallel.point_at(&end.into(), policy).unwrap());
    let range = exact(
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(start.into()),
            BezierParameter2::Exact(end.into()),
            policy,
        )
        .unwrap(),
    );
    let arc = BezierSplitFragment2::AnalyticParallel(exact(
        BezierParallelFragment2::try_new(parallel, range, policy).unwrap(),
    ));
    let chord = BezierSplitFragment2::AlgebraicChord(exact(
        BezierAlgebraicChord2::try_new(b.into(), a.into(), policy).unwrap(),
    ));
    let boundary = CurveRegionBoundaryLoop2::new(vec![arc, chord], policy).unwrap();
    let region = CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap();
    certified(region.regularized_region(policy).unwrap())
}
fn clip(policy: &CurveContext) -> CurveRegion2 {
    let corners = [
        Point2::new(q(3, 2), q(-1, 2)),
        Point2::new(q(5, 2), q(-1, 2)),
        Point2::new(q(5, 2), q(1, 2)),
        Point2::new(q(3, 2), q(1, 2)),
    ];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(corners[i].clone(), corners[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    certified(
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            policy,
        )
        .unwrap(),
    )
}
fn check(
    region: &CurveRegion2,
    point: Point2,
    expected: RegionPointLocation,
    policy: &CurveContext,
    checks: &mut usize,
) {
    let result = region.classify_point(&point, policy);
    println!("expected={expected:?} result={result:?}");
    assert_eq!(exact(certified(result.unwrap())), expected);
    *checks += 1;
}
fn main() {
    let mut checks = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for finite in [false, true] {
            println!("start finite={finite} policy={policy:?}");
            let cap = cap(finite, &policy);
            let clip = clip(&policy);
            let result = certified(cap.boolean_regions(&clip, &policy).unwrap());
            check(
                result.intersection(),
                p(2, 0),
                RegionPointLocation::Inside,
                &policy,
                &mut checks,
            );
            check(
                result.intersection(),
                Point2::new(2.into(), q(3, 4)),
                RegionPointLocation::Outside,
                &policy,
                &mut checks,
            );
            check(
                result.intersection(),
                Point2::new(2.into(), q(-1, 4)),
                RegionPointLocation::Boundary,
                &policy,
                &mut checks,
            );
            let paths = exact(certified(
                result.intersection().boundary_paths(&policy).unwrap(),
            ));
            let reentered = certified(
                CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                    &paths,
                    &vec![CurveRegionLoopRole::Material; paths.len()],
                    &vec![FillRule::NonZero; paths.len()],
                    &policy,
                )
                .unwrap(),
            );
            let normalized = certified(reentered.regularized_region(&policy).unwrap());
            check(
                &normalized,
                p(2, 0),
                RegionPointLocation::Inside,
                &policy,
                &mut checks,
            );
            check(
                &normalized,
                Point2::new(2.into(), q(-1, 4)),
                RegionPointLocation::Boundary,
                &policy,
                &mut checks,
            );
        }
    }
    println!("checks={checks}");
    assert_eq!(checks, 20);
}
