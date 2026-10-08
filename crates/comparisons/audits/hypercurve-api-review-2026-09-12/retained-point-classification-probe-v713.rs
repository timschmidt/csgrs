
#[cfg(test)]
mod retained_point_classification_probe_v713 {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture predicate: {reason:?}"),
        }
    }

    fn displaced_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        // Direction (3,4) gives the exact left unit normal (-4/5,3/5).
        let start = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
        let end = Point2::new(start.x() + Real::from(3), start.y() + Real::from(4));
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(start.into(), end.into(), policy).unwrap(),
        );
        let (image, _) = crate::BezierAlgebraicChordParallelPoint2::new_pair(
            chord,
            Real::one(),
            Real::zero(),
            Real::zero(),
            policy,
        );
        image.into()
    }

    fn image(point: &Point2, kind: usize, policy: &CurveContext) -> CurvePoint2 {
        match kind {
            0 => displaced_point(point, policy),
            1 => {
                let start = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
                let end = Point2::new(start.x() + Real::from(3), start.y() + Real::from(4));
                let parallel = RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2])
                    .unwrap()
                    .parallel_left(Real::one())
                    .unwrap();
                crate::BezierAnalyticParallelPoint2::new(
                    parallel,
                    BezierParameter2::Exact(Real::zero()),
                    policy,
                )
                .into()
            }
            2 => {
                let preimage = Point2::new(point.x() - Real::from(2), point.y() + Real::from(3));
                let translation = crate::Similarity2::try_from_real_affine(
                    Real::one(), Real::zero(), Real::zero(), Real::one(),
                    Real::from(2), Real::from(-3),
                ).unwrap();
                crate::bezier_offset::BezierSimilarityPoint2::new(
                    displaced_point(&preimage, policy), translation, policy,
                ).into()
            }
            3 => CurvePoint2::from_endpoint(
                std::sync::Arc::new(BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        point.clone(),
                        Point2::new(point.x() + Real::one(), point.y().clone()),
                        Point2::new(point.x() + Real::from(2), point.y().clone()),
                    )),
                }),
                true,
            ),
            _ => unreachable!(),
        }
    }

    fn check(kind: usize) {
        let coordinates = [(-1, 5), (1, 1), (3, 3), (5, 5), (9, 9), (11, 5),
            (0, 5), (2, 5), (4, 5), (6, 5), (8, 5), (10, 5), (0, 0), (2, 2), (4, 4)];
        let mut checked = 0;
        let mut failures = Vec::new();
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter().enumerate()
        {
            let images = coordinates.map(|(x, y)| image(&Point2::from_values(x, y), kind, &policy));
            assert!(images.iter().all(|point| point.coordinates().is_none()));
            for (rectangle_index, (low, high)) in [(0, 10), (2, 8), (4, 6)].into_iter().enumerate() {
                let vertices = [(low, low), (high, low), (high, high), (low, high)]
                    .map(|(x, y)| Point2::from_values(x, y));
                let contour = Contour2::try_new((0..4).map(|i| Segment2::Line(
                    LineSeg2::try_new(vertices[i].clone(), vertices[(i + 1) % 4].clone()).unwrap(),
                )).collect()).unwrap();
                let region = CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
                    .unwrap().value;
                assert_eq!(region.boundary_loops().len(), 1);
                for (point_index, ((x, y), point)) in coordinates.iter().zip(&images).enumerate() {
                    let expected = if *x < low || *x > high || *y < low || *y > high {
                        ContourPointLocation::Outside
                    } else if *x == low || *x == high || *y == low || *y == high {
                        ContourPointLocation::Boundary
                    } else {
                        ContourPointLocation::Inside
                    };
                    let actual = classify_point_evidence_against_retained_loop(&region, 0, point, &policy);
                    if !matches!(actual, Ok(Classification::Decided(location)) if location == expected) {
                        let code = match actual {
                            Ok(Classification::Decided(_)) => 0,
                            Ok(Classification::Uncertain(_)) => 1,
                            Err(_) => 2,
                        };
                        failures.push((policy_index, rectangle_index, point_index, code));
                    }
                    checked += 1;
                }
            }
        }
        eprintln!("retained point kind={kind} checks={checked} failures={}", failures.len());
        assert!(failures.is_empty(), "bounded point classification failures: {failures:?}");
    }

    #[test]
    fn chord_normal_points_classify_on_exact_rectangle_oracles() { check(0); }
    #[test]
    fn analytic_parallel_points_classify_on_exact_rectangle_oracles() { check(1); }
    #[test]
    fn similarity_points_classify_on_exact_rectangle_oracles() { check(2); }
    #[test]
    fn lazy_endpoint_points_classify_on_exact_rectangle_oracles() { check(3); }
}
