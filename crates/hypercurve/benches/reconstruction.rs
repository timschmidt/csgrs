use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Contour2, CurveResult, CurveString2, Point2, PolylineReconstructionOptions, Real,
};

fn r(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn line_samples(count: usize) -> Vec<Point2> {
    (0..count).map(|index| p(index as f64, 0.0)).collect()
}

fn semicircle_samples(count: usize) -> Vec<Point2> {
    (0..count)
        .map(|index| {
            let t = std::f64::consts::PI * index as f64 / (count - 1) as f64;
            p(
                1.0 + (std::f64::consts::PI + t).cos(),
                (std::f64::consts::PI + t).sin(),
            )
        })
        .collect()
}

fn rounded_rectangle_samples(samples_per_corner: usize) -> Vec<Point2> {
    let mut points = Vec::with_capacity(samples_per_corner * 4);
    let corners = [(4.0, 1.0), (4.0, 3.0), (1.0, 3.0), (1.0, 1.0)];
    let starts = [
        -std::f64::consts::FRAC_PI_2,
        0.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    ];

    for (corner, start) in corners.into_iter().zip(starts) {
        for sample in 0..samples_per_corner {
            let t = start + std::f64::consts::FRAC_PI_2 * sample as f64 / samples_per_corner as f64;
            points.push(p(corner.0 + t.cos(), corner.1 + t.sin()));
        }
    }
    points
}

fn bench_open_reconstruction(name: &str, points: &[Point2], iterations: u32) -> CurveResult<()> {
    let options = PolylineReconstructionOptions {
        min_arc_points: 3,
        distance_tolerance: 1e-8,
        ..PolylineReconstructionOptions::default()
    };
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let curve = CurveString2::reconstruct_from_polyline(points, options)?;
        total_segments += black_box(curve.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_closed_reconstruction(name: &str, points: &[Point2], iterations: u32) -> CurveResult<()> {
    let options = PolylineReconstructionOptions {
        min_arc_points: 3,
        distance_tolerance: 1e-8,
        ..PolylineReconstructionOptions::default()
    };
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let contour = Contour2::reconstruct_from_closed_polyline(points, options)?;
        total_segments += black_box(contour.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> CurveResult<()> {
    bench_open_reconstruction("reconstruct_collinear_256", &line_samples(256), 10_000)?;
    bench_open_reconstruction(
        "reconstruct_semicircle_129",
        &semicircle_samples(129),
        10_000,
    )?;
    bench_closed_reconstruction(
        "reconstruct_rounded_rectangle_128",
        &rounded_rectangle_samples(32),
        10_000,
    )?;
    Ok(())
}
