use super::*;
use hyperdrc::geometry::Polygon;

fn same(left: &Scalar, right: &Scalar) {
    assert_eq!(hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal));
}
fn construct(label: &str, center: [f64; 2], size: [f64; 2], angle: f64,
             expected_area: Scalar, expected_bounds: Option<[Scalar; 4]>, view_points: usize) {
    let inputs = center.into_iter().chain(size).chain([angle]).map(f64::to_bits).collect::<Vec<_>>();
    measure_value(label, 100, || rect_polygon(center, size, angle), |polygon: &Polygon| {
        same(&polygon.try_area().unwrap(), &expected_area);
        let bounds = polygon.try_bounds().unwrap();
        assert_eq!(bounds.is_some(), expected_bounds.is_some());
        if let (Some(actual), Some(expected)) = (&bounds, &expected_bounds) {
            for (actual, expected) in actual.iter().zip(expected) { same(actual, expected); }
        }
        assert_eq!(polygon.exterior().0.len(), view_points);
        // The input bits plus independent exact area/bounds assertions qualify
        // this witness even when its finite report view collapses or is empty.
        format!("{inputs:?}:exact-area-bounds:report-points={view_points}")
    });
}
pub(super) fn run() {
    let zero = Scalar::zero();
    let one = Scalar::one();
    let half = (one.clone() / Scalar::from(2)).unwrap();
    construct("rectangle_construct_unit", [0.0,0.0], [1.0,1.0], 0.0, one.clone(),
        Some([-half.clone(),-half.clone(),half.clone(),half.clone()]), 5);
    let huge: Scalar = "1e20".parse().unwrap();
    let h = Scalar::from(8192);
    construct("rectangle_construct_original_large", [1e20,0.0], [16_384.0,16_384.0], 0.0, Scalar::from(268_435_456),
        Some([&huge-&h,-h.clone(),&huge+&h,h.clone()]), 5);
    construct("rectangle_construct_unit_large", [1e20,0.0], [1.0,1.0], 0.0, one.clone(),
        Some([&huge-&half,-half.clone(),&huge+&half,half.clone()]), 5);
    let tiny = f64::from_bits(1);
    let side = Scalar::try_from(tiny).unwrap();
    let half_tiny = (&side / Scalar::from(2)).unwrap();
    construct("rectangle_construct_subnormal", [0.0,0.0], [tiny,tiny], 0.0, &side*&side,
        Some([-half_tiny.clone(),-half_tiny.clone(),half_tiny.clone(),half_tiny]), 4);
    let max = Scalar::try_from(f64::MAX).unwrap();
    let half_max = (&max / Scalar::from(2)).unwrap();
    construct("rectangle_construct_beyond_float", [f64::MAX,0.0], [f64::MAX,2.0], 0.0, &max*Scalar::from(2),
        Some([&max-&half_max,-one.clone(),&max+half_max,one.clone()]), 0);
    construct("rectangle_construct_quarter_turn", [1.0,2.0], [4.0,2.0], 90.0, Scalar::from(8),
        Some([zero.clone(),zero.clone(),Scalar::from(2),Scalar::from(4)]), 5);
    let root = Scalar::from(2).sqrt().unwrap();
    construct("rectangle_construct_eighth_turn", [0.0,0.0], [2.0,2.0], 45.0, Scalar::from(4),
        Some([-root.clone(),-root.clone(),root.clone(),root]), 5);
    let root = Scalar::from(3).sqrt().unwrap();
    let x = (&root+Scalar::from(3))*&half;
    let y = (one.clone()+&root*Scalar::from(3))*&half;
    construct("rectangle_construct_thirty_degrees", [0.0,0.0], [2.0,6.0], 30.0, Scalar::from(12),
        Some([-x.clone(),-y.clone(),x,y]), 5);
    construct("rectangle_construct_zero_side", [1e20,0.0], [0.0,2.0], 0.0, zero.clone(), None, 5);
    let width = Scalar::try_from(0.1).unwrap();
    let radius = &width*&half;
    let center = Scalar::try_from(0.95).unwrap();
    construct("rectangle_construct_finite_dyadic", [0.95,0.0], [0.1,0.1], 0.0, &width*&width,
        Some([&center-&radius,-radius.clone(),&center+&radius,radius]), 5);
    let polygon = rect_polygon([1e20,0.0], [16_384.0,16_384.0], 0.0);
    let expected = polygon.try_bounds().unwrap();
    measure_value("rectangle_warm_bounds_original_large", 1_000, || polygon.try_bounds().unwrap(), |bounds| {
        assert_eq!(bounds,&expected); "[1e20-8192,-8192,1e20+8192,8192]".into()
    });
    measure_value("rectangle_warm_area_original_large", 1_000, || polygon.try_area().unwrap(), |area| {
        same(area,&Scalar::from(268_435_456)); "268435456".into()
    });
}
