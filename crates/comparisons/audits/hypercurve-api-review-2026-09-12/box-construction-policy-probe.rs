use hypercurve::{Aabb2, Classification, CurveContext, Point2, Real};
fn main() {
    let epsilon = Real::one() - Real::from(2_i8).powi_i64(-607).unwrap().cos();
    let points = [Point2::from_values(0, 0), Point2::new(epsilon.clone(), Real::one())];
    let bounds = Aabb2::from_points(&points, &CurveContext::APPROXIMATE_512);
    match bounds {
        Classification::Decided(bounds) => {
            let order = epsilon.certified_cmp_until(bounds.max_x(), -2000).ordering();
            println!("input_x_vs_max_x={order:?}");
            println!("contains_input={:?}", bounds.contains_point(&points[1], &CurveContext::STRICT));
        }
        Classification::Uncertain(reason) => println!("bounds declined: {reason:?}"),
    }
}
