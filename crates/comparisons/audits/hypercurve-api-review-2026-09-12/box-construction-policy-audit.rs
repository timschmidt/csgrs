use hypercurve::{Aabb2, Classification, CurveContext, CurvePreviewOptions, CircularArc2, Point2, Real};
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
    let radius = Real::from(3_i8).sqrt().unwrap();
    let x = Real::from(2_i8).sqrt().unwrap();
    let arc = CircularArc2::try_from_center(
        Point2::new(x.clone(), Real::one()),
        Point2::new(-x, Real::one()),
        Point2::from_values(0, 0), false).unwrap();
    let preview = CurvePreviewOptions::try_strict(1e-12, 1e-12).unwrap();
    let bounds = preview.evaluate(|policy| Aabb2::from_arc(&arc, policy).unwrap());
    match bounds {
        Classification::Decided(bounds) => {
            println!("arc_radius_vs_max_y={:?}", radius.certified_cmp_until(bounds.max_y(), -2000).ordering());
            println!("contains_cardinal={:?}", bounds.contains_point(&Point2::new(Real::zero(), radius), &CurveContext::STRICT));
        }
        Classification::Uncertain(reason) => println!("arc bounds declined: {reason:?}"),
    }
    let epsilon = Real::one() - Real::from(2_i8).powi_i64(-619).unwrap().cos();
    let reversed = Aabb2::new_unchecked(Point2::new(epsilon.clone(), Real::zero()), Point2::from_values(0, 1));
    println!("reversed_box_accepted={:?}", reversed.has_valid_ordering(&CurveContext::APPROXIMATE_512));
    let first = Aabb2::from_point(Point2::from_values(0, 0));
    let second = Aabb2::from_point(Point2::new(epsilon, Real::zero()));
    println!("distinct_points_singleton={:?}", first.singleton_intersection(&second, &CurveContext::APPROXIMATE_512));
}
