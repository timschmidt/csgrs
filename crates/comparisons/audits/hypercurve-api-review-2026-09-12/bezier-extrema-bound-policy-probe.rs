use hypercurve::{Classification, CurveContext, Point2, QuadraticBezier2, Real};
fn main() {
    for policy in [CurveContext::APPROXIMATE_512, CurveContext::STRICT] {
        let exponent = 301 + i64::from(policy == CurveContext::STRICT);
        let delta = Real::one() - Real::from(2_i8).powi_i64(-exponent).unwrap().cos();
        let square = &delta * &delta;
        let curve = QuadraticBezier2::new(
            Point2::new(Real::zero(), square.clone()),
            Point2::new((Real::one() / Real::from(2_i8)).unwrap(), &square - &delta),
            Point2::new(Real::one(), &square - Real::from(2_i8) * &delta + Real::one()),
        );
        match curve.certified_bounds(&policy) {
            Classification::Decided(bounds) => {
                println!("{policy:?} min_y_vs_zero={:?}", bounds.min_y().certified_cmp_until(&Real::zero(), -3000).ordering());
                println!("contains_exact_minimum={:?}", bounds.contains_point(&Point2::new(delta, Real::zero()), &CurveContext::STRICT));
            }
            Classification::Uncertain(reason) => println!("{policy:?} declined: {reason:?}"),
        }
    }
}
