use std::{hint::black_box, time::Instant};

use hypercurve::{CircularArc2, Contour2, CurveRegion2, Point2, Segment2};
use hyperdrc::geometry::{MultiPolygon, multipolygon_to_shapes_scalar, rect_polygon};
use hyperdrc::{PcbRegion, PcbRegionExt};
use hyperreal::Real;

fn fixture(name: &str) -> (MultiPolygon, Real, usize) {
    match name {
        "rectangle" => (
            MultiPolygon(vec![rect_polygon([0.0, 0.0], [2.0, 3.0], 0.0)]),
            Real::one(),
            1,
        ),
        "rectangles_64" => (
            MultiPolygon(
                (0..64)
                    .map(|x| rect_polygon([f64::from(x) * 3.0, 0.0], [2.0, 3.0], 0.0))
                    .collect(),
            ),
            Real::one(),
            64,
        ),
        "native_circle" => {
            let right = Point2::from_values(1, 0);
            let left = Point2::from_values(-1, 0);
            let center = Point2::from_values(0, 0);
            let contour = Contour2::try_new(vec![
                Segment2::Arc(
                    CircularArc2::try_from_center(
                        right.clone(),
                        left.clone(),
                        center.clone(),
                        false,
                    )
                    .unwrap(),
                ),
                Segment2::Arc(CircularArc2::try_from_center(left, right, center, false).unwrap()),
            ])
            .unwrap();
            let region = PcbRegion::new(
                CurveRegion2::try_from_native_material_contours(vec![contour]).unwrap(),
                None,
            );
            (region.to_multipolygon(), Real::from(3), 1)
        }
        _ => unreachable!(),
    }
}

fn observe(polygons: &MultiPolygon, minimum_area: &Real) -> (usize, [f64; 4]) {
    let shapes = multipolygon_to_shapes_scalar(polygons, minimum_area);
    let bounds = polygons.bounding_rect().unwrap();
    (
        shapes.len(),
        [
            bounds.min().x,
            bounds.min().y,
            bounds.max().x,
            bounds.max().y,
        ],
    )
}

fn main() {
    let iterations = std::env::var("DRC_MEASUREMENT_ITERATIONS")
        .ok()
        .map(|s| s.parse::<usize>().unwrap())
        .unwrap_or(100);
    assert!(iterations > 0);
    for name in ["rectangle", "native_circle", "rectangles_64"] {
        let (polygons, minimum_area, count) = fixture(name);
        let expected = observe(&polygons, &minimum_area);
        assert_eq!(expected.0, count);
        let report_bytes =
            serde_json::to_vec(&multipolygon_to_shapes_scalar(&polygons, &minimum_area)).unwrap();
        // Stable byte checksum is compared across the two executables, outside
        // the timed loops. This is an output check, not a cryptographic digest.
        let checksum = report_bytes.iter().fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
        });
        let mut last = expected;
        let start = Instant::now();
        for _ in 0..iterations {
            last = black_box(observe(black_box(&polygons), black_box(&minimum_area)));
        }
        let warm = start.elapsed().as_nanos();
        assert_eq!(last, expected);
        println!(
            "{{\"fixture\":\"{name}\",\"mode\":\"warm\",\"iterations\":{iterations},\"elapsed_ns\":{warm},\"rows\":{count},\"output_bytes\":{},\"checksum\":{checksum},\"bounds\":{:?}}}",
            report_bytes.len(),
            expected.1
        );
        let start = Instant::now();
        for _ in 0..iterations {
            let (polygons, minimum_area, _) = fixture(name);
            last = black_box(observe(&polygons, &minimum_area));
        }
        let cold = start.elapsed().as_nanos();
        assert_eq!(last, expected);
        println!(
            "{{\"fixture\":\"{name}\",\"mode\":\"construct_and_measure\",\"iterations\":{iterations},\"elapsed_ns\":{cold},\"rows\":{count},\"output_bytes\":{},\"checksum\":{checksum},\"bounds\":{:?}}}",
            report_bytes.len(),
            expected.1
        );
    }
}
