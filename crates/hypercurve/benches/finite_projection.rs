use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Classification, CubicBezier2, Curve2, CurveContext, CurvePath2, CurveRegion2,
    FiniteProjectionOptions, LineSeg2, Point2, RationalBezier2, Real,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn rational_path() -> CurvePath2 {
    CurvePath2::try_new(vec![Curve2::from(
        RationalBezier2::try_new(
            vec![
                point(0, 0),
                point(1, 3),
                point(2, -3),
                point(3, 3),
                point(4, 0),
            ],
            vec![
                Real::one(),
                Real::from(2),
                Real::from(3),
                Real::from(2),
                Real::one(),
            ],
        )
        .unwrap(),
    )])
    .unwrap()
}

fn cubic_region() -> CurveRegion2 {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
        Curve2::from(CubicBezier2::new(
            point(4, 4),
            point(3, 6),
            point(1, 6),
            point(0, 4),
        )),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::STRICT)
        .unwrap()
        .into_value()
}

fn measure(name: &str, iterations: u32, mut workload: impl FnMut() -> usize) {
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
}

fn main() {
    let iterations: u32 = std::env::var("HYPERCURVE_PROJECTION_BENCH_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(200);
    let options = FiniteProjectionOptions::try_new(1.0e-3).unwrap();
    let rational = rational_path();
    let region = cubic_region();
    let policy = CurveContext::STRICT;

    measure("curve_path_rational_projection", iterations, || {
        rational
            .project_to_finite_polyline(&options, &policy)
            .unwrap()
            .into_value()
            .points()
            .len()
    });
    measure(
        "curve_region_exact_profile_projection",
        iterations.saturating_mul(5),
        || match region
            .project_to_finite_profiles_exact(&options, &policy)
            .unwrap()
            .into_value()
        {
            Classification::Decided(profiles) => profiles
                .iter()
                .map(|profile| profile.material().points().len())
                .sum(),
            Classification::Uncertain(reason) => {
                panic!("exact projection benchmark became uncertain: {reason:?}")
            }
        },
    );
    measure(
        "curve_region_materialized_boundary_paths",
        iterations.saturating_mul(5_000),
        || match region
            .materialized_boundary_paths(&policy)
            .unwrap()
            .into_value()
        {
            Classification::Decided(paths) => paths.iter().map(|path| path.curves().len()).sum(),
            Classification::Uncertain(reason) => {
                panic!("materialized boundary benchmark became uncertain: {reason:?}")
            }
        },
    );
    measure(
        "curve_region_curve_path_projection",
        iterations.saturating_mul(5_000),
        || match region
            .project_to_finite_curve_paths(&policy)
            .unwrap()
            .into_value()
        {
            Classification::Decided(paths) => paths.iter().map(|path| path.curves().len()).sum(),
            Classification::Uncertain(reason) => {
                panic!("curve-path projection benchmark became uncertain: {reason:?}")
            }
        },
    );
}
