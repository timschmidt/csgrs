#![no_main]

use hypercurve::{
    Curve2, CurveContext, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2, QuadraticBezier2,
    Real, RegionPointLocation,
};
use libfuzzer_sys::fuzz_target;

fn half() -> Real {
    (Real::from(1) / Real::from(2)).unwrap()
}

/// The graph `y = k x^2` on `x` in `[0, 1]`, leaving the origin horizontally.
fn parabola(k: &Real) -> QuadraticBezier2 {
    QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(half(), Real::zero()),
        Point2::new(Real::one(), k.clone()),
    )
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    // Two distinct curvatures with a common horizontal tangent at the origin.
    // The origin is a branch vertex that only second-order tangent ordering
    // resolves; the sign bit reflects the lens below the axis.
    let first = i32::from(data[0] % 16) + 1;
    let second = i32::from(data[1] % 16) + 1;
    if first == second {
        return;
    }
    let sign = if data[2] & 1 == 0 { 1 } else { -1 };
    let (a, b) = (Real::from(sign * first), Real::from(sign * second));
    let policy = if data[2] & 2 == 0 {
        CurveContext::STRICT
    } else {
        CurveContext::APPROXIMATE_512
    };
    let lower = Curve2::from(parabola(&a));
    let upper = Curve2::from(parabola(&b))
        .reversed(&policy)
        .expect("reversal is exact")
        .into_value();
    let closing = LineSeg2::try_new(
        Point2::new(Real::one(), a.clone()),
        Point2::new(Real::one(), b.clone()),
    )
    .expect("distinct curvatures give a nonzero closing segment");
    let path = CurvePath2::try_new(vec![lower, closing.into(), upper])
        .expect("the lens boundary is connected");
    let region = under(&policy, || {
        CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd)
    })
    .expect("a simple tangent lens must be admitted");

    // At x = 1/2 the curves sit at a/4 and b/4, so (a+b)/8 is strictly inside.
    let eighth = (Real::from(1) / Real::from(8)).unwrap();
    let inside = Point2::new(half(), (&a + &b) * &eighth);
    let outside = Point2::new(half(), Real::from(-sign * 32));
    for (point, expected) in [
        (inside, RegionPointLocation::Inside),
        (outside, RegionPointLocation::Outside),
    ] {
        let location = under(&policy, || region.classify_point(&point.into()))
            .expect("classification completes");
        assert_eq!(location, expected);
    }
});

/// Runs a principal exact operation under `policy`: directly under STRICT,
/// and otherwise inside `hypercurve::provisional`.
fn under<T>(policy: &CurveContext, operation: impl FnOnce() -> T) -> T {
    if *policy == CurveContext::STRICT {
        operation()
    } else {
        hypercurve::provisional(operation).into_unverified()
    }
}
