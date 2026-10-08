use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    CurveContext, CurveResult, NurbsCurve2, Point2, Real, Similarity2,
    finite_polyline_vertex_centroid, finite_ring_signed_area, triangulate_finite_rings,
    try_finite_polyline_vertex_centroid, try_finite_ring_signed_area,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn measure(name: &str, iterations: u32, mut workload: impl FnMut() -> usize) {
    let mut run = || {
        let started = Instant::now();
        let mut checksum = 0_usize;
        for _ in 0..iterations {
            checksum = checksum.wrapping_add(black_box(workload()));
        }
        let elapsed = started.elapsed();
        println!(
            "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
            elapsed / iterations
        );
    };

    #[cfg(feature = "dispatch-trace")]
    {
        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(|| {
            hyperreal::dispatch_trace::record(
                "hypercurve-benchmark",
                "api-surface",
                "recorded-workload",
            );
            run();
        });
        let correlation = hyperreal::dispatch_trace::take_trace().correlation_summary();
        assert!(
            correlation.dispatch_events > 0,
            "{name} did not emit an API-surface path trace"
        );
        println!("{name}: correlation={correlation:?}");
    }

    #[cfg(not(feature = "dispatch-trace"))]
    run();
}

fn main() -> CurveResult<()> {
    let finite_ring = [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0], [0.0, 0.0]];
    measure("finite_projection_measurements", 100_000, || {
        let area = finite_ring_signed_area(black_box(&finite_ring));
        let checked_area = try_finite_ring_signed_area(black_box(&finite_ring)).unwrap();
        let centroid = finite_polyline_vertex_centroid(black_box(&finite_ring)).unwrap();
        let checked_centroid = try_finite_polyline_vertex_centroid(black_box(&finite_ring))
            .unwrap()
            .unwrap();
        (area == checked_area) as usize
            + (centroid == checked_centroid) as usize
            + centroid[0] as usize
            + centroid[1] as usize
    });

    let transform = Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(5), r(7))?;
    let point = p(3, 4);
    measure("exact_similarity_transform", 100_000, || {
        let transformed = transform.transform_point(black_box(&point));
        transformed.x().to_f64_lossy().is_some() as usize
            + transformed.y().to_f64_lossy().is_some() as usize
    });

    measure("nurbs_global_interpolation", 10_000, || {
        let curve = NurbsCurve2::interpolate_uniform(
            2,
            vec![p(0, 0), p(2, 2), p(4, 0)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        curve.control_points().len()
    });

    let material = [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]];
    let hole = [[2.0, 2.0], [2.0, 4.0], [6.0, 4.0], [6.0, 2.0]];
    measure("finite_ring_triangulation", 10_000, || {
        triangulate_finite_rings(
            black_box(&material),
            &[black_box(&hole)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
        .len()
    });
    Ok(())
}
