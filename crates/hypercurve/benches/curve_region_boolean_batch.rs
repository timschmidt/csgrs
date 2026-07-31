use std::hint::black_box;
use std::time::Instant;

use hypercurve::{BooleanOp, BulgeVertex2, Contour2, CurveContext, CurveRegion2, Point2, Real};

fn point(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn rectangle(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(min_x, min_y), Real::zero()),
        BulgeVertex2::new(point(max_x, min_y), Real::zero()),
        BulgeVertex2::new(point(max_x, max_y), Real::zero()),
        BulgeVertex2::new(point(min_x, max_y), Real::zero()),
    ])
    .unwrap()
}

fn region_weight(region: &CurveRegion2) -> usize {
    region
        .boundary_loops()
        .iter()
        .map(|boundary| boundary.len())
        .sum()
}

fn measure(operation: &mut impl FnMut() -> usize, iterations: u32) -> (u128, usize) {
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        checksum ^= black_box(operation());
    }
    (started.elapsed().as_nanos(), checksum)
}

fn main() {
    let policy = CurveContext::STRICT;
    let first =
        CurveRegion2::try_from_native_material_contours(vec![rectangle(0, 0, 4, 4)], &policy)
            .unwrap()
            .into_value();
    let second =
        CurveRegion2::try_from_native_material_contours(vec![rectangle(2, 0, 6, 4)], &policy)
            .unwrap()
            .into_value();
    let iterations = std::env::var("HYPERCURVE_CURVE_REGION_BATCH_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(1_000_u32);

    let mut independent = || {
        [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ]
        .into_iter()
        .map(|operation| {
            first
                .boolean_region(&second, operation, &policy)
                .unwrap()
                .into_value()
        })
        .map(|region| region_weight(&region))
        .sum()
    };
    let mut shared = || {
        let regions = first
            .boolean_regions(&second, &policy)
            .unwrap()
            .into_value();
        region_weight(regions.union())
            + region_weight(regions.intersection())
            + region_weight(regions.difference())
            + region_weight(regions.xor())
    };

    black_box(independent());
    black_box(shared());
    match std::env::var("HYPERCURVE_CURVE_REGION_BATCH_MODE").as_deref() {
        Ok("independent") => {
            let (ns, checksum) = measure(&mut independent, iterations);
            println!(
                "curve_region_boolean_four_independent: {iterations} iterations, total_ns={ns}, ns_per_iter={}, checksum={checksum}",
                ns / u128::from(iterations)
            );
            return;
        }
        Ok("shared") => {
            let (ns, checksum) = measure(&mut shared, iterations);
            println!(
                "curve_region_boolean_shared_all_four: {iterations} iterations, total_ns={ns}, ns_per_iter={}, checksum={checksum}",
                ns / u128::from(iterations)
            );
            return;
        }
        Ok(mode) => panic!("unknown batch benchmark mode {mode}"),
        Err(_) => {}
    }
    let shared_first = std::env::var_os("HYPERCURVE_CURVE_REGION_BATCH_SHARED_FIRST").is_some();
    let ((independent_ns, independent_checksum), (shared_ns, shared_checksum)) = if shared_first {
        let shared = measure(&mut shared, iterations);
        let independent = measure(&mut independent, iterations);
        (independent, shared)
    } else {
        let independent = measure(&mut independent, iterations);
        let shared = measure(&mut shared, iterations);
        (independent, shared)
    };
    println!(
        "curve_region_boolean_four_independent: {iterations} iterations, total_ns={independent_ns}, ns_per_iter={}, checksum={independent_checksum}",
        independent_ns / u128::from(iterations)
    );
    println!(
        "curve_region_boolean_shared_all_four: {iterations} iterations, total_ns={shared_ns}, ns_per_iter={}, checksum={shared_checksum}",
        shared_ns / u128::from(iterations)
    );
}
