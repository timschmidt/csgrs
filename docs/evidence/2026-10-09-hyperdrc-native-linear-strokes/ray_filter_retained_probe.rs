use hypercurve::{
    Curve2, CurveGeometry2, CurveParameterRange2, CurvePath2, CurveRegion2, ExactCurveError, FillRule, Point2, PredicatePolicy,
    QuadraticBezier2, Real, RegionPointLocation, evaluate_under,
};

fn classify_lens_center(origin: &str, scale: &str) -> Result<RegionPointLocation, ExactCurveError> {
    let origin: Real = origin.parse().unwrap();
    let scale: Real = scale.parse().unwrap();
    let map = |x: i32, y: i32| {
        Point2::new(&origin + &scale * Real::from(x), &origin + &scale * Real::from(y))
    };
    // With t=u-1, these sources trace the same exact lens on u in [1,2].
    // Keeping the non-unit source range exercises retained query carriers.
    let range = CurveParameterRange2::try_new(Real::one().into(), Real::from(2).into())?;
    let path = CurvePath2::try_new(vec![
        Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(-3, 4), map(-2, 1), map(-1, 0))), range.clone())?,
        Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(3, -4), map(2, -1), map(1, 0))), range)?,
    ])?;
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)?;
    eprintln!("retained_fragments={}", region.has_algebraic_fragments());
    region.classify_point(&map(0, 0).into())
}

fn main() {
    let mut failures = 0;
    for (policy_name, policy) in [("STRICT", PredicatePolicy::STRICT), ("APPROXIMATE_512", PredicatePolicy::APPROXIMATE_512)] {
        for (origin, scale) in [("0", "1"), ("1e308", "1e307"), ("-1e308", "1e307")] {
            eprintln!("starting {policy:?} origin={origin} scale={scale}");
            let result = evaluate_under(policy, || classify_lens_center(origin, scale)).certified();
            let valid = matches!(result, Some(Ok(RegionPointLocation::Inside)));
            let actual = match &result {
                Some(Ok(RegionPointLocation::Inside)) => "Inside",
                Some(Ok(RegionPointLocation::Outside)) => "Outside",
                Some(Ok(RegionPointLocation::Boundary)) => "Boundary",
                Some(Err(_)) => "Error",
                None => "Uncertified",
            };
            eprintln!("result={result:?}");
            println!("{{\"policy\":\"{policy_name}\",\"origin\":\"{origin}\",\"scale\":\"{scale}\",\"expected\":\"Inside\",\"actual\":\"{actual}\",\"passed\":{valid}}}");
            failures += usize::from(!valid);
        }
    }
    assert_eq!(failures, 0, "public affine lens center classifications must remain exact");
}
