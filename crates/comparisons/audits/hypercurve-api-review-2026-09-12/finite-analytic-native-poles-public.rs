use hypercurve::*;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!(
            "uncertain: {}",
            matches!(other, Classification::Uncertain(_))
        ),
    }
}
fn certified<T>(value: CurveOutcome<T>) -> T {
    assert_eq!(value.certainty, CurveCertainty::Certified);
    value.value
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
    let source = paths[0].curves()[0].clone();
    if reverse {
        certified(source.reversed(policy).unwrap())
    } else {
        source
    }
}
fn source(native: bool, displaced: bool, reverse: bool, policy: &CurveContext) -> Curve2 {
    // P(t)=(-t/(1-2t),-t/(1-2t)), t in [0,1/4].
    // The native chart is P(u/4), u in [0,1]. The unused pole is t=1/2.
    let (end, weight, stop) = if native {
        (Point2::new(q(-1, 2), q(-1, 2)), q(1, 2), Real::one())
    } else {
        (p(1, 1), -Real::one(), q(1, 4))
    };
    let source = RationalBezier2::try_new(vec![p(0, 0), end], vec![Real::one(), weight]).unwrap();
    let distance = if displaced {
        q(1, 4) * Real::from(2).sqrt().unwrap()
    } else {
        Real::zero()
    };
    let parallel = source.parallel_left(distance).unwrap();
    let first = decided(parallel.point_at(&Real::zero(), policy).unwrap());
    let last = decided(parallel.point_at(&stop, policy).unwrap());
    let range = decided(
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(stop),
            policy,
        )
        .unwrap(),
    );
    let fragment = decided(BezierParallelFragment2::try_new(parallel, range, policy).unwrap());
    curve(
        BezierSplitFragment2::AnalyticParallel(fragment),
        first,
        last,
        reverse,
        policy,
    )
}
fn horizontal(displaced: bool, parallel: bool, reverse: bool, policy: &CurveContext) -> Curve2 {
    let y = if displaced { q(-5, 12) } else { q(-1, 6) };
    let a = Point2::new((-1).into(), y.clone());
    let b = Point2::new(1.into(), y.clone());
    let source = QuadraticBezier2::new(a.clone(), Point2::new(Real::zero(), y), b.clone());
    let fragment = if parallel {
        BezierSplitFragment2::AnalyticParallel(decided(
            BezierParallelFragment2::try_new(
                source.parallel_left(Real::zero()).unwrap(),
                decided(
                    BezierParameterRange2::try_new(
                        BezierParameter2::Exact(Real::zero()),
                        BezierParameter2::Exact(Real::one()),
                        policy,
                    )
                    .unwrap(),
                ),
                policy,
            )
            .unwrap(),
        ))
    } else {
        BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::Quadratic(source),
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            reversed: false,
            start_image: None,
            end_image: None,
        }
    };
    curve(fragment, a, b, reverse, policy)
}
fn main() {
    let mut cases = 0;
    let mut contacts = 0;
    let mut failures = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for native in [false, true] {
            for displaced in [false, true] {
                for parallel in [false, true] {
                    for reverse_a in [false, true] {
                        for reverse_b in [false, true] {
                            let a = source(native, displaced, reverse_a, &policy);
                            let b = horizontal(displaced, parallel, reverse_b, &policy);
                            let expected: CurvePoint2 = if displaced {
                                Point2::new(q(1, 12), q(-5, 12))
                            } else {
                                Point2::new(q(-1, 6), q(-1, 6))
                            }
                            .into();
                            for swapped in [false, true] {
                                cases += 1;
                                let (a, b) = if swapped { (&b, &a) } else { (&a, &b) };
                                let result = a.intersect_curve(b, &policy);
                                let label = format!(
                                    "native={native} displaced={displaced} parallel={parallel} reverse_a={reverse_a} reverse_b={reverse_b} swapped={swapped} policy={policy:?}"
                                );
                                match result {
                                    Ok(outcome)
                                        if outcome.certainty == CurveCertainty::Certified
                                            && outcome.value.is_complete()
                                            && outcome.value.contacts().len() == 1
                                            && outcome.value.overlaps().is_empty()
                                            && outcome.value.parameter_components().is_empty() =>
                                    {
                                        let contact = &outcome.value.contacts()[0];
                                        assert_eq!(
                                            certified(
                                                contact.point().coincides_with(&expected, &policy)
                                            ),
                                            Classification::Decided(true),
                                            "{label}"
                                        );
                                        for (source, location) in
                                            [(a, contact.first()), (b, contact.second())]
                                        {
                                            let parameter =
                                                decided(location.parameter(&policy).unwrap());
                                            let point = certified(
                                                source.point_at(&parameter, &policy).unwrap(),
                                            );
                                            assert_eq!(
                                                certified(point.coincides_with(&expected, &policy)),
                                                Classification::Decided(true),
                                                "{label}"
                                            );
                                        }
                                        contacts += 1;
                                        println!("{label}: exact contact and location replay");
                                    }
                                    other => {
                                        failures += 1;
                                        println!("{label}: {other:?}");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    println!("{{\"cases\":{cases},\"contacts\":{contacts},\"failures\":{failures}}}");
    assert_eq!(cases, 128);
    assert_eq!(failures, 0);
}
