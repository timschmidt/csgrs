use hyperdrc::geometry::{MultiPolygon, circle_polygon, polygons_to_profile, rect_polygon};
use hyperdrc::{PcbRegion, PcbRegionExt, Scalar};
use std::hint::black_box;
use std::time::Instant;

fn native_area(view: &MultiPolygon<f64>) -> Scalar {
    hypercurve::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
        view.0.iter().fold(Scalar::zero(), |area, polygon| {
            area + polygon.exact_region().filled_area().unwrap().unwrap()
        })
    })
    .certified()
    .unwrap()
}

fn fingerprint(view: &MultiPolygon<f64>) -> (usize, usize, u64) {
    let mut points = 0;
    let mut hash = 0xcbf29ce484222325_u64;
    for polygon in &view.0 {
        for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
            for coordinate in &ring.0 {
                points += 1;
                for value in [coordinate.x, coordinate.y] {
                    for byte in value.to_bits().to_le_bytes() {
                        hash ^= u64::from(byte);
                        hash = hash.wrapping_mul(0x100000001b3);
                    }
                }
            }
        }
    }
    (view.0.len(), points, hash)
}

fn measure(
    label: &str,
    iterations: usize,
    expected_area: &Scalar,
    mut query: impl FnMut() -> MultiPolygon<f64>,
) {
    let first = query();
    assert_eq!(
        hyperlimit::compare_reals(
            &native_area(&first),
            expected_area,
            hyperlimit::PredicatePolicy::STRICT
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let expected = fingerprint(&first);
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    let last = query();
    assert_eq!(fingerprint(&last), expected);
    assert_eq!(
        hyperlimit::compare_reals(
            &native_area(&last),
            expected_area,
            hyperlimit::PredicatePolicy::STRICT
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    println!(
        "{label}\t{iterations}\t{elapsed}\t{}\t{}\t{}",
        expected.0, expected.1, expected.2
    );
}

fn main() {
    for samples in [17, 1_000_000] {
        let circle = polygons_to_profile(vec![circle_polygon([0.0, 0.0], 1.0, samples)], None);
        let iterations = if samples == 17 { 1_000 } else { 10 };
        assert_eq!(
            circle.try_to_multipolygon().unwrap().0[0].exterior().0.len(),
            samples + 1
        );
        measure(
            &format!("authored_circle_{samples}_warm"),
            iterations,
            &Scalar::pi(),
            || circle.try_to_multipolygon().unwrap(),
        );
    }
    let authored = polygons_to_profile(vec![circle_polygon([0.0, 0.0], 1.0, 17)], None);
    let native = (*authored).clone();
    measure("native_circle_cold_drc_cache", 100, &Scalar::pi(), || {
        PcbRegion::new(native.clone(), None)
            .try_to_multipolygon()
            .unwrap()
    });
    let warm = PcbRegion::new(native, None);
    measure("native_circle_warm", 1_000, &Scalar::pi(), || {
        warm.try_to_multipolygon().unwrap()
    });
    let field = polygons_to_profile(
        (0..64)
            .map(|index| rect_polygon([f64::from(index) * 4.0, 0.0], [1.0, 1.0], 0.0))
            .collect(),
        None,
    );
    let native = (*field).clone();
    measure("components64_cold_drc_cache", 25, &Scalar::from(64), || {
        PcbRegion::new(native.clone(), None)
            .try_to_multipolygon()
            .unwrap()
    });
    let warm = PcbRegion::new(native, None);
    measure("components64_warm", 1_000, &Scalar::from(64), || {
        warm.try_to_multipolygon().unwrap()
    });
}
