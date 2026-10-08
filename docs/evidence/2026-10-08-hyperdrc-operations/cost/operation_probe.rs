use hyperdrc::geometry::{circle_polygon, polygons_to_profile, rect_polygon};
use hyperdrc::{PcbRegion, PcbRegionExt, Scalar};
use std::hint::black_box;
use std::time::Instant;

fn witness(region: &PcbRegion, expected_area: &Scalar) -> String {
    let area = hypercurve::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
        region.filled_area().unwrap().unwrap()
    })
    .certified()
    .unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&area, expected_area, hyperlimit::PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert!(region.try_bounding_box().unwrap().is_some());
    let view = region.try_to_multipolygon().unwrap();
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
    format!("{}:{points}:{hash}", view.0.len())
}

fn measure_region(label: &str, expected_area: Scalar, mut query: impl FnMut() -> PcbRegion) {
    let expected = witness(&query(), &expected_area);
    let iterations = 100;
    let started = Instant::now();
    for _ in 0..iterations {
        let region = query();
        black_box(region.try_bounding_box().unwrap());
        black_box(region);
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(witness(&query(), &expected_area), expected);
    println!("operation_{label}\t{iterations}\t{elapsed}\t{expected}");
}

fn main() {
    let left = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)], None);
    let right = polygons_to_profile(vec![rect_polygon([1.0, 0.0], [2.0, 2.0], 0.0)], None);
    measure_region("union_and_bounds", 6.into(), || {
        left.try_union(&right).unwrap()
    });
    measure_region("intersection_and_bounds", 2.into(), || {
        left.try_intersection(&right).unwrap()
    });
    measure_region("difference_and_bounds", 2.into(), || {
        left.try_difference(&right).unwrap()
    });
    measure_region("xor_and_bounds", 4.into(), || left.try_xor(&right).unwrap());
    let authored = polygons_to_profile(vec![circle_polygon([0.0, 0.0], 1.0, 17)], None);
    let circle = PcbRegion::new((*authored).clone(), None);
    measure_region(
        "circle_offset_and_bounds",
        Scalar::from(4) * Scalar::pi(),
        || circle.offset(Scalar::one()).unwrap(),
    );
    let drill = hyperdrc::kicad::DrillFeature {
        location: [Scalar::zero(), Scalar::zero()],
        diameter: 2.into(),
        plated: false,
        net: None,
    };
    measure_region("drill_circle_and_bounds", Scalar::pi(), || {
        hyperdrc::checks::drill::drills_to_region(
            std::slice::from_ref(&drill),
            "probe",
            "operation-cost",
        )
        .unwrap()
    });
    let point = [Scalar::zero(), Scalar::zero()];
    let query = || {
        circle
            .classify_xy(point[0].clone(), point[1].clone())
            .unwrap()
    };
    assert_eq!(query(), hypercurve::RegionPointLocation::Inside);
    let iterations = 1_000;
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(query(), hypercurve::RegionPointLocation::Inside);
    println!("operation_classification\t{iterations}\t{elapsed}\tinside");
    let query = || circle.try_bounding_box().unwrap().unwrap();
    let expected = query();
    assert_eq!(expected.mins.x, Scalar::from(-1));
    assert_eq!(expected.mins.y, Scalar::from(-1));
    assert_eq!(expected.maxs.x, Scalar::one());
    assert_eq!(expected.maxs.y, Scalar::one());
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(query(), expected);
    println!("operation_bounds\t{iterations}\t{elapsed}\t-1,-1,1,1");
}
