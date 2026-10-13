use super::*;
use hyperdrc::geometry::{
    Polygon, chamfered_rect_polygon, rounded_rect_polygon, trapezoid_polygon,
};

fn scalar(text: &str) -> Scalar {
    text.parse().unwrap()
}
fn same(left: &Scalar, right: &Scalar) {
    assert_eq!(
        hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}
fn rounded_area(width: Scalar, height: Scalar, radius: Scalar) -> Scalar {
    width * height - (Scalar::from(4) - Scalar::pi()) * &radius * radius
}
fn bounds(x: Scalar, y: Scalar, center: Scalar) -> Option<[Scalar; 4]> {
    Some([&center - &x, -y.clone(), center + x, y])
}
fn view(polygon: &Polygon) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for coordinate in &polygon.exterior().0 {
        for value in [coordinate.x, coordinate.y] {
            assert!(value.is_finite());
            for byte in value.to_bits().to_le_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
            }
        }
    }
    format!("{}:{hash}", polygon.exterior().0.len())
}
fn construct(
    label: &str,
    inputs: &[f64],
    area: Scalar,
    expected_bounds: Option<[Scalar; 4]>,
    query: impl FnMut() -> Polygon,
) {
    let input_bits = inputs.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
    measure_value(label, 100, query, |polygon| {
        same(&polygon.try_area().unwrap(), &area);
        let actual = polygon.try_bounds().unwrap();
        assert_eq!(actual.is_some(), expected_bounds.is_some());
        if let (Some(actual), Some(expected)) = (&actual, &expected_bounds) {
            for (a, b) in actual.iter().zip(expected) {
                same(a, b);
            }
        }
        format!("{input_bits:?}:exact-area-bounds:view={}", view(polygon))
    });
}
pub(super) fn run() {
    let zero = Scalar::zero();
    let one = Scalar::one();
    let half = scalar("0.5");
    let area = rounded_area(2.into(), 1.into(), scalar("0.25"));
    for samples in [4, 128] {
        construct(
            &format!("pad_round_construct_samples_{samples}"),
            &[0.0, 0.0, 2.0, 1.0, 0.25, 0.0, samples as f64],
            area.clone(),
            bounds(one.clone(), half.clone(), zero.clone()),
            || rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, samples),
        );
    }
    let huge = scalar("1e20");
    let area_huge = rounded_area(16384.into(), 16384.into(), 4096.into());
    construct(
        "pad_round_construct_original_large",
        &[1e20, 0.0, 16384.0, 16384.0, 4096.0, 0.0, 8.0],
        area_huge,
        bounds(8192.into(), 8192.into(), huge.clone()),
        || rounded_rect_polygon([1e20, 0.0], [16384.0, 16384.0], 4096.0, 0.0, 8),
    );
    let tiny = f64::from_bits(1);
    let t = Scalar::try_from(tiny).unwrap();
    let tiny_area = rounded_area(&t * Scalar::from(2), &t * Scalar::from(4), t.clone());
    construct(
        "pad_round_construct_subnormal",
        &[0.0, 0.0, 2.0 * tiny, 4.0 * tiny, tiny, 0.0, 8.0],
        tiny_area,
        bounds(t.clone(), &t * Scalar::from(2), zero.clone()),
        || rounded_rect_polygon([0.0, 0.0], [2.0 * tiny, 4.0 * tiny], tiny, 0.0, 8),
    );
    construct(
        "pad_round_construct_capsule",
        &[0.0, 0.0, 4.0, 2.0, 99.0, 0.0, 8.0],
        Scalar::from(4) + Scalar::pi(),
        bounds(2.into(), 1.into(), zero.clone()),
        || rounded_rect_polygon([0.0, 0.0], [4.0, 2.0], 99.0, 0.0, 8),
    );
    construct(
        "pad_round_construct_circle",
        &[0.0, 0.0, 2.0, 2.0, 99.0, 45.0, 8.0],
        Scalar::pi(),
        bounds(1.into(), 1.into(), zero.clone()),
        || rounded_rect_polygon([0.0, 0.0], [2.0, 2.0], 99.0, 45.0, 8),
    );
    construct(
        "pad_round_construct_zero_radius",
        &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 8.0],
        2.into(),
        bounds(1.into(), half.clone(), zero.clone()),
        || rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.0, 0.0, 8),
    );
    let max = Scalar::try_from(f64::MAX).unwrap();
    let max_half = (&max / Scalar::from(2)).unwrap();
    let max_radius = Scalar::try_from(f64::MAX / 4.0).unwrap();
    construct(
        "pad_round_construct_beyond_float",
        &[f64::MAX, 0.0, f64::MAX, f64::MAX, f64::MAX / 4.0, 0.0, 4.0],
        rounded_area(max.clone(), max.clone(), max_radius),
        bounds(max_half.clone(), max_half, max),
        || rounded_rect_polygon([f64::MAX, 0.0], [f64::MAX, f64::MAX], f64::MAX / 4.0, 0.0, 4),
    );

    let c = Scalar::try_from(0.2).unwrap();
    construct(
        "pad_chamfer_construct_selected",
        &[0.0, 0.0, 2.0, 1.0, 0.2, 1.0, 0.0, 1.0, 0.0, 0.0],
        Scalar::from(2) - &c * &c,
        bounds(1.into(), half.clone(), zero.clone()),
        || {
            chamfered_rect_polygon(
                [0.0, 0.0],
                [2.0, 1.0],
                0.2,
                [true, false, true, false],
                0.0,
            )
        },
    );
    construct(
        "pad_chamfer_construct_all",
        &[0.0, 0.0, 2.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 0.0],
        scalar("1.5"),
        bounds(1.into(), half.clone(), zero.clone()),
        || chamfered_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.5, [true; 4], 0.0),
    );
    construct(
        "pad_chamfer_construct_diamond",
        &[0.0, 0.0, 2.0, 2.0, 99.0, 1.0, 1.0, 1.0, 1.0, 0.0],
        2.into(),
        bounds(1.into(), 1.into(), zero.clone()),
        || chamfered_rect_polygon([0.0, 0.0], [2.0, 2.0], 99.0, [true; 4], 0.0),
    );
    construct(
        "pad_chamfer_construct_original_large",
        &[1e20, 0.0, 16384.0, 16384.0, 4096.0, 1.0, 1.0, 1.0, 1.0, 0.0],
        234881024.into(),
        bounds(8192.into(), 8192.into(), huge.clone()),
        || chamfered_rect_polygon([1e20, 0.0], [16384.0, 16384.0], 4096.0, [true; 4], 0.0),
    );
    let t_half = (&t / Scalar::from(2)).unwrap();
    construct(
        "pad_chamfer_construct_subnormal",
        &[0.0, 0.0, tiny, tiny, tiny, 1.0, 1.0, 1.0, 1.0, 0.0],
        (&t * &t / Scalar::from(2)).unwrap(),
        bounds(t_half.clone(), t_half.clone(), zero.clone()),
        || chamfered_rect_polygon([0.0, 0.0], [tiny, tiny], tiny, [true; 4], 0.0),
    );

    construct(
        "pad_trapezoid_construct_half_delta",
        &[0.0, 0.0, 2.0, 1.0, 0.5, 0.25, 0.0],
        2.into(),
        bounds(scalar("1.125"), scalar("0.75"), zero.clone()),
        || trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [0.5, 0.25], 0.0),
    );
    construct(
        "pad_trapezoid_construct_original_large",
        &[1e20, 0.0, 16384.0, 16384.0, 2048.0, 4096.0, 0.0],
        268435456.into(),
        bounds(10240.into(), 9216.into(), huge.clone()),
        || trapezoid_polygon([1e20, 0.0], [16384.0, 16384.0], [2048.0, 4096.0], 0.0),
    );
    construct(
        "pad_trapezoid_construct_triangle",
        &[0.0, 0.0, 2.0, 1.0, 1.0, 0.0, 0.0],
        2.into(),
        bounds(1.into(), 1.into(), zero.clone()),
        || trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [1.0, 0.0], 0.0),
    );
    construct(
        "pad_trapezoid_construct_both_limits",
        &[0.0, 0.0, 2.0, 1.0, 1.0, 2.0, 0.0],
        2.into(),
        bounds(2.into(), 1.into(), zero.clone()),
        || trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [1.0, 2.0], 0.0),
    );
    construct(
        "pad_trapezoid_construct_subnormal",
        &[0.0, 0.0, tiny, tiny, 0.0, 0.0, 90.0],
        &t * &t,
        bounds(t_half.clone(), t_half, zero.clone()),
        || trapezoid_polygon([0.0, 0.0], [tiny, tiny], [0.0, 0.0], 90.0),
    );
    measure_value(
        "pad_trapezoid_invalid_delta",
        100,
        || trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [1.5, 0.0], 0.0),
        |polygon| {
            let error = polygon.try_area().unwrap_err();
            assert!(
                error
                    .detail
                    .contains("trapezoid delta exceeds the opposite size")
            );
            assert!(polygon.try_bounds().is_err());
            "size=2,1:delta=1.5,0:retained-source-failure".into()
        },
    );
    construct(
        "pad_trapezoid_construct_zero_side",
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        zero.clone(),
        None,
        || trapezoid_polygon([0.0, 0.0], [0.0, 1.0], [0.0, 0.0], 0.0),
    );
    // Twenty construction rows above; native measurements below are warm.
    for samples in [4, 128] {
        let polygon = rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, samples);
        measure_value(
            &format!("pad_round_warm_area_samples_{samples}"),
            1000,
            || polygon.try_area().unwrap(),
            |value| {
                same(value, &area);
                "2-(4-pi)/16".into()
            },
        );
        let expected = bounds(one.clone(), half.clone(), zero.clone());
        measure_value(
            &format!("pad_round_warm_bounds_samples_{samples}"),
            1000,
            || polygon.try_bounds().unwrap(),
            |value| {
                assert_eq!(value, &expected);
                "[-1,-0.5,1,0.5]".into()
            },
        );
    }
    let polygon =
        chamfered_rect_polygon([1e20, 0.0], [16384.0, 16384.0], 4096.0, [true; 4], 0.0);
    measure_value(
        "pad_chamfer_warm_area_original_large",
        1000,
        || polygon.try_area().unwrap(),
        |value| {
            same(value, &Scalar::from(234881024));
            "234881024".into()
        },
    );
    let polygon = trapezoid_polygon([1e20, 0.0], [16384.0, 16384.0], [2048.0, 4096.0], 0.0);
    let expected = bounds(10240.into(), 9216.into(), huge);
    measure_value(
        "pad_trapezoid_warm_bounds_original_large",
        1000,
        || polygon.try_bounds().unwrap(),
        |value| {
            assert_eq!(value, &expected);
            "[1e20-10240,-9216,1e20+10240,9216]".into()
        },
    );
}
