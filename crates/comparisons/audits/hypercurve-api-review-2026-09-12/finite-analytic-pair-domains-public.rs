use hypercurve::*;

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(v) => v,
        Classification::Uncertain(e) => panic!("{e:?}"),
    }
}
fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}
fn range(start: i32, end: i32, policy: &CurveContext) -> BezierParameterRange2 {
    decided(
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(start.into()),
            BezierParameter2::Exact(end.into()),
            policy,
        )
        .unwrap(),
    )
}
fn curve(
    fragment: BezierSplitFragment2,
    start: Point2,
    end: Point2,
    reverse: bool,
    policy: &CurveContext,
) -> Curve2 {
    let via = p(7, -3);
    let chord = |a: Point2, b: Point2| {
        BezierSplitFragment2::AlgebraicChord(decided(
            BezierAlgebraicChord2::try_new(a.into(), b.into(), policy).unwrap(),
        ))
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![fragment, chord(end, via.clone()), chord(via, start)],
        policy,
    )
    .unwrap();
    let region = CurveRegion2::new(vec![boundary]).unwrap();
    let paths = decided(certified(region.boundary_paths(policy).unwrap()));
    let curve = paths[0].curves()[0].clone();
    if reverse {
        certified(curve.reversed(policy).unwrap())
    } else {
        curve
    }
}
fn analytic(
    source: QuadraticBezier2,
    distance: Real,
    start: i32,
    end: i32,
    reverse: bool,
    policy: &CurveContext,
) -> Curve2 {
    let parallel = source.parallel_left(distance).unwrap();
    let a = decided(parallel.point_at(&start.into(), policy).unwrap());
    let b = decided(parallel.point_at(&end.into(), policy).unwrap());
    let fragment = decided(
        BezierParallelFragment2::try_new(parallel, range(start, end, policy), policy).unwrap(),
    );
    curve(
        BezierSplitFragment2::AnalyticParallel(fragment),
        a,
        b,
        reverse,
        policy,
    )
}
fn cap(finite: bool, reverse: bool, policy: &CurveContext) -> Curve2 {
    if finite {
        analytic(
            QuadraticBezier2::new(p(0, 4), Point2::new(q(1, 2), 2.into()), p(1, 1)),
            q(-1, 4),
            1,
            3,
            reverse,
            policy,
        )
    } else {
        analytic(
            QuadraticBezier2::new(p(1, 1), p(2, -1), p(3, 1)),
            q(-1, 4),
            0,
            1,
            reverse,
            policy,
        )
    }
}
fn vertical(finite: bool, parallel: bool, reverse: bool, policy: &CurveContext) -> Curve2 {
    let x = if parallel { q(9, 4) } else { Real::from(2) };
    let point = |y: Real| Point2::new(x.clone(), y);
    let source = if finite {
        QuadraticBezier2::new(point(0.into()), point(q(1, 2)), point(1.into()))
    } else {
        QuadraticBezier2::new(point((-1).into()), point(0.into()), point(1.into()))
    };
    let (start, end) = if finite { (-1, 1) } else { (0, 1) };
    if parallel {
        return analytic(source, q(1, 4), start, end, reverse, policy);
    }
    curve(
        BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::Quadratic(source),
            start: BezierParameter2::Exact(start.into()),
            end: BezierParameter2::Exact(end.into()),
            start_image: None,
            end_image: None,
            reversed: false,
        },
        p(2, -1),
        p(2, 1),
        reverse,
        policy,
    )
}
fn main() {
    let mut cases = 0;
    let mut failures = 0;
    let mut exact_contacts = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for first_finite in [false, true] {
            for second_finite in [false, true] {
                for parallel in [false, true] {
                    for first_reverse in [false, true] {
                        for second_reverse in [false, true] {
                            let first = cap(first_finite, first_reverse, &policy);
                            let second = vertical(second_finite, parallel, second_reverse, &policy);
                            for swapped in [false, true] {
                                cases += 1;
                                let (first, second) = if swapped {
                                    (&second, &first)
                                } else {
                                    (&first, &second)
                                };
                                let result = first.intersect_curve(second, &policy);
                                let label = format!(
                                    "first_finite={first_finite} second_finite={second_finite} parallel={parallel} first_reverse={first_reverse} second_reverse={second_reverse} swapped={swapped} policy={policy:?}"
                                );
                                match result {
                                    Ok(outcome)
                                        if outcome.certainty == CurveCertainty::Certified =>
                                    {
                                        let contacts = outcome.value;
                                        println!(
                                            "{label}: complete={} contacts={} overlaps={} blockers={:?}",
                                            contacts.is_complete(),
                                            contacts.contacts().len(),
                                            contacts.overlaps().len(),
                                            contacts.blockers()
                                        );
                                        if !contacts.is_complete()
                                            || contacts.contacts().len() != 1
                                            || !contacts.overlaps().is_empty()
                                        {
                                            failures += 1;
                                            continue;
                                        }
                                        let contact = &contacts.contacts()[0];
                                        let expected: CurvePoint2 =
                                            Point2::new(2.into(), q(-1, 4)).into();
                                        assert_eq!(
                                            certified(
                                                contact.point().coincides_with(&expected, &policy)
                                            ),
                                            Classification::Decided(true)
                                        );
                                        for (source, location) in
                                            [(first, contact.first()), (second, contact.second())]
                                        {
                                            let parameter =
                                                decided(location.parameter(&policy).unwrap());
                                            let point = certified(
                                                source.point_at(&parameter, &policy).unwrap(),
                                            );
                                            assert_eq!(
                                                certified(point.coincides_with(&expected, &policy)),
                                                Classification::Decided(true)
                                            );
                                        }
                                        exact_contacts += 1;
                                    }
                                    other => {
                                        println!("{label}: {other:?}");
                                        failures += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    println!("{{\"cases\":{cases},\"exact_contacts\":{exact_contacts},\"failures\":{failures}}}");
    assert_eq!(cases, 128);
    assert_eq!(failures, 0);
}
