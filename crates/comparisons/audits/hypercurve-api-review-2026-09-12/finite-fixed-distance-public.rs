use hypercurve::*;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn exact<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("{reason:?}"),
    }
}
fn certified<T>(value: CurveOutcome<T>) -> T {
    assert_eq!(value.certainty, CurveCertainty::Certified);
    value.value
}
fn same(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
    assert_eq!(
        certified(first.coincides_with(second, policy)),
        Classification::Decided(true)
    );
}
fn cap(chart: usize, policy: &CurveContext) -> Curve2 {
    // All four charts cover P(t)=(-1/8+t^2,t), 0<=t<=1/8.
    // The rational exterior chart has t=(s-2)/(2s-1), s in [2,5/2].
    // Its genuine pole at s=1/2 is outside the requested interval.
    let exterior = chart & 1 != 0;
    let rational = chart & 2 != 0;
    let (parallel, start, end) = if rational {
        let (points, weights, start, end) = if exterior {
            (
                vec![
                    Point2::new(q(31, 8), 2.into()),
                    Point2::new(q(-17, 8), q(1, 2)),
                    Point2::new(q(7, 8), (-1).into()),
                ],
                vec![1.into(), (-1).into(), 1.into()],
                Real::from(2),
                q(5, 2),
            )
        } else {
            (
                vec![
                    Point2::new(q(-1, 8), Real::zero()),
                    Point2::new(q(-1, 8), q(1, 16)),
                    Point2::new(q(-7, 64), q(1, 8)),
                ],
                vec![9.into(), 12.into(), 16.into()],
                Real::zero(),
                Real::one(),
            )
        };
        (
            RationalBezier2::try_new(points, weights)
                .unwrap()
                .parallel_left(q(1, 64))
                .unwrap(),
            start,
            end,
        )
    } else {
        let (source, start, end) = if exterior {
            (
                QuadraticBezier2::new(
                    Point2::new(q(31, 8), (-2).into()),
                    Point2::new(q(15, 8), q(-3, 2)),
                    Point2::new(q(7, 8), (-1).into()),
                ),
                Real::from(2),
                q(17, 8),
            )
        } else {
            (
                QuadraticBezier2::new(
                    Point2::new(q(-1, 8), Real::zero()),
                    Point2::new(q(-1, 8), q(1, 16)),
                    Point2::new(q(-7, 64), q(1, 8)),
                ),
                Real::zero(),
                Real::one(),
            )
        };
        (source.parallel_left(q(1, 64)).unwrap(), start, end)
    };
    let first = exact(parallel.point_at(&start, policy).unwrap());
    let last = exact(parallel.point_at(&end, policy).unwrap());
    let range = exact(
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(start),
            BezierParameter2::Exact(end),
            policy,
        )
        .unwrap(),
    );
    let fragment = BezierSplitFragment2::AnalyticParallel(exact(
        BezierParallelFragment2::try_new(parallel, range, policy).unwrap(),
    ));
    let via = p(8, -4);
    let chord = |a: Point2, b: Point2| {
        BezierSplitFragment2::AlgebraicChord(exact(
            BezierAlgebraicChord2::try_new(a.into(), b.into(), policy).unwrap(),
        ))
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![fragment, chord(last, via.clone()), chord(via, first)],
        policy,
    )
    .unwrap();
    let region = CurveRegion2::new(vec![boundary]).unwrap();
    exact(certified(region.boundary_paths(policy).unwrap()))[0].curves()[0].clone()
}
fn main() {
    let chart: usize = std::env::args().nth(1).expect("chart 0..3").parse().unwrap();
    let repeat = std::env::args().nth(2).is_some();
    let (mut cases, mut successes, mut replays, mut failures) = (0, 0, 0, 0);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let curved = cap(chart, &policy);
        let corner = Point2::new(q(-9, 64), Real::zero());
        same(&curved.start(), &corner.clone().into(), &policy);
        let line = LineSeg2::try_new(
            Point2::new(corner.x() - Real::one(), Real::zero()), corner.clone(),
        ).unwrap();
        let original = CurvePath2::try_new(vec![line.into(), curved.clone()]).unwrap();
        for reversed in [false, true] {
            cases += 1;
            let path = if reversed { certified(original.reversed(&policy).unwrap()) } else { original.clone() };
            println!("begin chart={chart} reversed={reversed} repeat={repeat} policy={policy:?}");
            let result = path.chamfer_vertex_by_setbacks(1, q(1,128), q(1,128), CurveCornerMode2::TrimOnly, &policy);
            let modified = match result {
                Ok(CurveOutcome { certainty: CurveCertainty::Certified, value: CurveCornerSolutions2::Unique(path) }) => path,
                other => { failures += 1; println!("first chamfer: {other:?}"); continue; }
            };
            println!("first chamfer returned");
            assert_eq!(modified.curves().len(), 3);
            same(&modified.start(), &path.start(), &policy);
            same(&modified.end(), &path.end(), &policy);
            let chord = &modified.curves()[1];
            assert_eq!(chord.family(), CurveFamily2::Line);
            let expected_line_cut: CurvePoint2 = Point2::new(q(-19,128), Real::zero()).into();
            same(&if reversed { chord.end() } else { chord.start() }, &expected_line_cut, &policy);
            for adjacent in modified.curves().windows(2) { same(&adjacent[0].end(), &adjacent[1].start(), &policy); replays += 1; }
            let survivor = &modified.curves()[if reversed {0} else {2}];
            for endpoint in [survivor.parameter_domain().start(), survivor.parameter_domain().end()] {
                let first = certified(survivor.point_at(endpoint, &policy).unwrap());
                let original_curve = &path.curves()[if reversed {0} else {1}];
                same(&first, &certified(original_curve.point_at(endpoint, &policy).unwrap()), &policy);
                replays += 1;
            }
            if repeat {
                println!("begin repeated chamfer");
                let index = if reversed {1} else {2};
                match modified.chamfer_vertex_by_setbacks(index, q(1,256), q(1,256), CurveCornerMode2::TrimOnly, &policy) {
                    Ok(CurveOutcome { certainty: CurveCertainty::Certified, value: CurveCornerSolutions2::Unique(next) }) => {
                        assert_eq!(next.curves().len(), 4);
                        same(&next.start(), &path.start(), &policy);
                        same(&next.end(), &path.end(), &policy);
                        for adjacent in next.curves().windows(2) { same(&adjacent[0].end(), &adjacent[1].start(), &policy); replays += 1; }
                    }
                    other => { failures += 1; println!("repeated chamfer: {other:?}"); continue; }
                }
            }
            successes += 1;
            println!("complete chart={chart} reversed={reversed} policy={policy:?}");
        }
    }
    println!("{{\"cases\":{cases},\"successes\":{successes},\"point_replays\":{replays},\"failures\":{failures}}}");
    assert_eq!(failures, 0);
}
