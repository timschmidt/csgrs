use hypercurve::{
    Curve2, CurvePath2, CurveRegion2, ExactCurveError, FillRule, Point2, PredicatePolicy,
    QuadraticBezier2, Real, RegionPointLocation, evaluate_under,
};

fn classify_lens_center(origin: &str, scale: &str) -> Result<RegionPointLocation, ExactCurveError> {
    let origin: Real = origin.parse().unwrap();
    let scale: Real = scale.parse().unwrap();
    let map = |x: i32, y: i32| {
        Point2::new(&origin + &scale * Real::from(x), &origin + &scale * Real::from(y))
    };
    // Relative to origin and scale, these are x=2t-1 and
    // y=+/-2t(1-t). Their common center is strictly inside the lens.
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(map(-1, 0), map(0, -1), map(1, 0))),
        Curve2::from(QuadraticBezier2::new(map(1, 0), map(0, 1), map(-1, 0))),
    ])?;
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)?;
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
