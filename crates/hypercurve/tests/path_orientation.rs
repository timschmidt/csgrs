use hypercurve::*;

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn line(a: Point2, b: Point2) -> Curve2 {
    LineSeg2::try_new(a, b).unwrap().into()
}

fn polygon(points: &[(i32, i32)]) -> CurvePath2 {
    CurvePath2::try_new(
        (0..points.len())
            .map(|i| {
                let a = points[i];
                let b = points[(i + 1) % points.len()];
                line(p(a.0, a.1), p(b.0, b.1))
            })
            .collect(),
    )
    .unwrap()
}

#[test]
fn simple_loop_orientation_uses_winding_without_a_represented_integral() {
    for nurbs in [false, true] {
        let controls = vec![p(0, 0), p(1, -1), p(2, -1), p(3, -1), p(4, -1), p(5, 0)];
        let weights = vec![1.into(), 2.into(), 3.into(), 4.into(), 5.into(), 1.into()];
        let curve = if nurbs {
            Curve2::try_nurbs(
                5,
                controls,
                weights,
                [vec![Real::from(2); 6], vec![Real::from(4); 6]].concat(),
            )
            .unwrap()
        } else {
            RationalBezier2::try_new(controls, weights).unwrap().into()
        };
        let path = CurvePath2::try_new(vec![
            curve,
            line(p(5, 0), p(5, 2)),
            line(p(5, 2), p(0, 2)),
            line(p(0, 2), p(0, 0)),
        ])
        .unwrap();
        for (path, expected) in [
            (path.clone(), CurveLoopOrientation2::Counterclockwise),
            (path.reversed().unwrap(), CurveLoopOrientation2::Clockwise),
        ] {
            assert!(
                path.boundary_loop()
                    .unwrap()
                    .signed_area()
                    .unwrap()
                    .is_none()
            );
            for policy in [
                PredicatePolicy::STRICT,
                PredicatePolicy::APPROXIMATE_512,
                PredicatePolicy::STRICT,
            ] {
                let result = evaluate_under(policy, || path.simple_loop_orientation());
                assert_eq!(result.certainty(), CurveCertainty::Certified);
                assert_eq!(result.into_unverified().unwrap(), expected);
            }
        }
    }
}

#[test]
fn orientation_keeps_selected_boolean_boundaries() {
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4)).into(),
        line(p(2, 4), p(-2, 4)),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap();
    let clip = CurveRegion2::try_from_boundary_paths(
        &[polygon(&[(-3, 2), (3, 2), (3, 5), (-3, 5)])],
        FillRule::EvenOdd,
    )
    .unwrap();
    let result = region.boolean_region(&clip, BooleanOp::Difference).unwrap();
    assert!(result.has_algebraic_fragments());
    let path = CurvePath2::try_new(result.boundary_loops()[0].curves().to_vec()).unwrap();
    assert!(path.start().coordinates().is_none());
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
    assert_eq!(
        path.reversed().unwrap().simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Clockwise
    );
    assert!(path.start().coordinates().is_none());
}

#[test]
fn orientation_does_not_normalize_away_invalid_path_topology() {
    for points in [
        vec![(0, 0), (2, 2), (0, 2), (2, 0)],
        vec![(0, 0), (4, 0), (4, 4), (0, 4), (0, 0), (-1, 0)],
        vec![(0, 0), (2, 0), (1, 0)],
        vec![(0, 0), (2, 0), (2, 2), (0, 0), (-2, 0), (-2, -2)],
    ] {
        assert!(
            polygon(&points).simple_loop_orientation().is_err(),
            "{points:?}"
        );
    }
    let open = CurvePath2::try_new(vec![line(p(0, 0), p(1, 1))]).unwrap();
    assert!(matches!(
        open.simple_loop_orientation(),
        Err(ExactCurveError::Invalid {
            cause: CurveError::OpenCurvePath,
            ..
        })
    ));
    // A failed proof cannot poison a later valid query.
    assert_eq!(
        polygon(&[(0, 0), (2, 0), (2, 2), (0, 2)])
            .simple_loop_orientation()
            .unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
}
