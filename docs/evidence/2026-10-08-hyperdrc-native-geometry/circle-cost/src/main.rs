use std::{hint::black_box, time::Instant};

use hypercurve::RegionPointLocation;
use hyperdrc::geometry::{Polygon, circle_polygon};
use hyperlimit::PredicatePolicy;
use hyperreal::Real;

fn observation(polygon: &Polygon) -> (usize, Real, [Real; 4]) {
    hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
        let region = polygon.exact_region();
        let area = region.filled_area().unwrap().unwrap();
        let bounds = region.bounds().unwrap().unwrap();
        (
            polygon.exterior().0.len(),
            area,
            [
                bounds.min_x().clone(),
                bounds.min_y().clone(),
                bounds.max_x().clone(),
                bounds.max_y().clone(),
            ],
        )
    })
    .certified()
    .unwrap()
}

fn check(value: &(usize, Real, [Real; 4]), segments: usize) {
    assert_eq!(value.0, segments + 1);
    assert_eq!(
        hyperlimit::compare_reals(&value.1, &Real::pi(), PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(value.2, [0, 0, 2, 2].map(Real::from));
}

fn checksum(polygon: &Polygon) -> u64 {
    polygon
        .exterior()
        .0
        .iter()
        .flat_map(|point| [point.x, point.y])
        .flat_map(|value| value.to_bits().to_le_bytes())
        .fold(0xcbf29ce484222325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
        })
}

fn via_observation() -> usize {
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/via.kicad_pcb"
    )))
    .unwrap();
    for feature in &board.copper {
        let x: Real = "0.999".parse().unwrap();
        let y: Real = "0.03".parse().unwrap();
        assert!(&x * &x + &y * &y < Real::one());
        let location = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
            feature
                .region
                .classify_point(&hypercurve::Point2::new(x, y).into())
                .unwrap()
        })
        .certified()
        .unwrap();
        assert_eq!(location, RegionPointLocation::Inside);
    }
    board.copper.len()
}

fn main() {
    let iterations = std::env::var("HYPERDRC_CIRCLE_ITERATIONS")
        .ok()
        .map(|value| value.parse::<usize>().unwrap())
        .unwrap_or(100);
    assert!(iterations > 0);
    for segments in [16_usize, 48, 128, 1_000_000] {
        let polygon = circle_polygon([1.0, 1.0], 1.0, segments);
        let mut last = observation(&polygon);
        check(&last, segments);
        let expected_checksum = checksum(&polygon);
        let start = Instant::now();
        for _ in 0..iterations {
            last = black_box(observation(black_box(&polygon)));
        }
        let elapsed = start.elapsed().as_nanos();
        check(&last, segments);
        println!(
            "{{\"fixture\":\"circle_{segments}\",\"mode\":\"warm_measure\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed},\"report_points\":{},\"report_coordinate_bytes\":{},\"checksum\":{expected_checksum}}}",
            segments + 1,
            (segments + 1) * 16
        );
        drop(polygon);
        let start = Instant::now();
        for _ in 0..iterations {
            let polygon = circle_polygon([1.0, 1.0], 1.0, black_box(segments));
            last = black_box(observation(&polygon));
        }
        let elapsed = start.elapsed().as_nanos();
        check(&last, segments);
        let repeated = circle_polygon([1.0, 1.0], 1.0, segments);
        assert_eq!(checksum(&repeated), expected_checksum);
        println!(
            "{{\"fixture\":\"circle_{segments}\",\"mode\":\"construct_and_measure\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed},\"report_points\":{},\"report_coordinate_bytes\":{},\"checksum\":{expected_checksum}}}",
            segments + 1,
            (segments + 1) * 16
        );
    }
    assert_eq!(via_observation(), 2);
    let start = Instant::now();
    for _ in 0..iterations {
        assert_eq!(black_box(via_observation()), 2);
    }
    let elapsed = start.elapsed().as_nanos();
    println!(
        "{{\"fixture\":\"source_via\",\"mode\":\"load_and_classify\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed},\"copper_features\":2}}"
    );
}
