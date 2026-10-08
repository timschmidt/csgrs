    #[test]
    fn planar_path_nesting_preserves_generated_offset_endpoints() {
        let policy = CurveContext::STRICT;
        let point = |x, y| CurvePoint2::new(r(x), r(y));
        let line = |first, second| Curve2::from(LineSeg2::try_new(first, second).unwrap());
        let cap = CurvePath2::try_new(vec![
            hypercurve::QuadraticBezier2::new(point(-1, 1), point(0, -1), point(1, 1)).into(),
            line(point(1, 1), point(-1, 1)),
        ])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths(&[cap], &policy)
            .unwrap()
            .into_value();
        let offset = region
            .offset(
                (Real::one() / r(4)).unwrap(),
                &hypercurve::OffsetCornerStyle2::Round,
                &policy,
            )
            .unwrap();
        assert_eq!(offset.certainty, hypercurve::CurveCertainty::Certified);
        let paths = offset.value.boundary_paths(&policy).unwrap();
        assert_eq!(paths.certainty, hypercurve::CurveCertainty::Certified);
        let Classification::Decided(mut paths) = paths.value else {
            panic!("the exact offset retains its boundary paths");
        };
        assert_eq!(paths.len(), 1);
        let mut curves = paths.pop().unwrap().curves().to_vec();
        let retained_start = curves
            .iter()
            .position(|curve| curve.start().coordinates().is_none())
            .expect("the offset has a generated endpoint");
        curves.rotate_left(retained_start);
        let hole = CurvePath2::try_new(curves).unwrap();
        assert!(hole.start().coordinates().is_none());
        let rectangle = |low, high| {
            let points = [point(low, low), point(high, low), point(high, high), point(low, high)];
            CurvePath2::try_new((0..4)
                .map(|i| line(points[i].clone(), points[(i + 1) % 4].clone()))
                .collect()).unwrap()
        };
        assert!(validate_planar_path_nesting(&rectangle(-3, 3), std::slice::from_ref(&hole)).is_ok());
        assert!(matches!(
            validate_planar_path_nesting(&rectangle(5, 8), &[hole]),
            Err(ConstructionError::HoleOutside)
        ));
    }
