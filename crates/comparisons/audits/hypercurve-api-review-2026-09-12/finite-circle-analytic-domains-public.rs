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
fn circle(policy: &CurveContext) -> Curve2 {
    let path = CurvePath2::try_new(vec![
        LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
        QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
    ])
    .unwrap();
    let CurveCornerSolutions2::Unique(path) = certified(
        path.fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, policy)
            .unwrap(),
    ) else {
        panic!("unique retained fillet")
    };
    let circle = path.curves()[1].clone();
    println!(
        "generated circle: family={:?}, retained={}",
        circle.family(),
        circle.geometry().is_none()
    );
    assert_eq!(circle.family(), CurveFamily2::CircularArc);
    assert!(circle.geometry().is_none());
    circle
}
fn cap(exterior: bool, policy: &CurveContext) -> Curve2 {
    // P(t)=(-1/8+t^2,t), 0<=t<=1/8. The exterior chart is P(s-2)
    // and the compact native chart is P(u/8). The normal displacement is 1/64.
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
    let parallel = source.parallel_left(q(1, 64)).unwrap();
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
fn oriented(curve: &Curve2, reverse: bool, policy: &CurveContext) -> Curve2 {
    if reverse {
        certified(curve.reversed(policy).unwrap())
    } else {
        curve.clone()
    }
}
fn main() {
    let (mut cases, mut contacts, mut replays, mut failures) = (0, 0, 0, 0);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let original_circle = circle(&policy);
        for exterior in [false, true] {
            let original_cap = cap(exterior, &policy);
            for reverse_circle in [false, true] {
                for reverse_cap in [false, true] {
                    let circle = oriented(&original_circle, reverse_circle, &policy);
                    let cap = oriented(&original_cap, reverse_cap, &policy);
                    for swapped in [false, true] {
                        cases += 1;
                        let (first, second) = if swapped {
                            (&cap, &circle)
                        } else {
                            (&circle, &cap)
                        };
                        let label = format!(
                            "exterior={exterior} reverse_circle={reverse_circle} reverse_cap={reverse_cap} swapped={swapped} policy={policy:?}"
                        );
                        match first.intersect_curve(second, &policy) {
                            Ok(outcome)
                                if outcome.certainty == CurveCertainty::Certified
                                    && outcome.value.is_complete()
                                    && outcome.value.contacts().len() == 1
                                    && outcome.value.overlaps().is_empty()
                                    && outcome.value.parameter_components().is_empty() =>
                            {
                                let contact = &outcome.value.contacts()[0];
                                // Independent monotonicity proves one transverse
                                // contact on the retained lower circle arc.
                                if let Some(cross) = contact.tangent_cross_sign() {
                                    assert_eq!(
                                        cross,
                                        if reverse_circle ^ reverse_cap ^ swapped {
                                            RealSign::Negative
                                        } else {
                                            RealSign::Positive
                                        },
                                        "{label}"
                                    );
                                }
                                for (source, location) in
                                    [(first, contact.first()), (second, contact.second())]
                                {
                                    let parameter = exact(location.parameter(&policy).unwrap());
                                    same(
                                        &certified(source.point_at(&parameter, &policy).unwrap()),
                                        contact.point(),
                                        &policy,
                                    );
                                    replays += 1;
                                }
                                contacts += 1;
                                println!(
                                    "{label}: exact transverse contact and public location replay"
                                );
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
    println!(
        "{{\"cases\":{cases},\"contacts\":{contacts},\"point_replays\":{replays},\"failures\":{failures}}}"
    );
    assert_eq!(cases, 32);
    assert_eq!(failures, 0);
}
