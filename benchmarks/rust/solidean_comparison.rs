#[path = "../support/harness.rs"]
#[allow(dead_code)]
mod support;

#[path = "../support/solidean.rs"]
mod solidean;

use std::hint::black_box;
use std::time::Instant;

use csgrs::TriangleMesh;
use solidean::Request;
use support::{Config, Measurement, print_header};

const SUITE: &str = "published-solidean";

fn main() {
    print_header();
    if !solidean::enabled() {
        eprintln!("Solidean benchmarks are opt-in; set SOLIDEAN_BENCH=1 to run them");
        return;
    }

    let config = Config::from_env();
    run_case(
        &config,
        solidean::first_request,
        "iterated_difference",
        "primitives_A_minus_B_to_J",
        Some((13.4224, 2.3513)),
    );
    run_case(
        &config,
        solidean::terrain_request,
        "iterated_difference",
        "terrain_0.2_-y_224",
        Some((6.1648, 0.5477)),
    );
    run_case(
        &config,
        solidean::cube_grid_request,
        "iterated_union_difference",
        "checker_grid_n10_1999",
        Some((0.0, 0.0)),
    );

    for level in dome_levels() {
        let expected = match level {
            100 => Some((64_106.545_0, 274_099.866_1)),
            250 => Some((25_505.199_0, 197_234.460_5)),
            1_000 => Some((23_790.045_1, 192_212.033_0)),
            5_000 => Some((23_041.886_4, 189_335.568_8)),
            _ => None,
        };
        run_case(
            &config,
            || solidean::dome_request(level),
            "iterated_difference",
            &format!("dome_carve_{level}"),
            expected,
        );
    }
}

fn run_case<F>(
    config: &Config,
    load: F,
    benchmark: &str,
    case: &str,
    expected: Option<(f64, f64)>,
) where
    F: FnOnce() -> Request,
{
    if !config.selects(SUITE, benchmark, case) {
        return;
    }
    let request = load();

    config.run_with_elapsed(SUITE, benchmark, case, 1, || {
        let start = Instant::now();
        let output = request.execute();
        let elapsed = start.elapsed();
        let measurement = Measurement::new(
            request.boolean_count() as u64,
            output.triangles.len() as u64,
            solidean::output_checksum(&output),
        );
        validate(&output, case, expected);
        black_box((elapsed, measurement))
    });
    eprintln!(
        "Solidean {case} ({}/{}): {} Boolean operations, {} input triangles",
        request.id,
        request.case_id,
        request.boolean_count(),
        request.input_triangles()
    );
}

fn validate(mesh: &TriangleMesh, case: &str, expected: Option<(f64, f64)>) {
    let (area, volume) = solidean::area_and_volume(mesh);
    assert!(
        area.is_finite() && volume.is_finite(),
        "{case} produced non-finite mass properties"
    );
    if let Some((expected_area, expected_volume)) = expected {
        assert_close(area, expected_area, case, "surface area");
        assert_close(volume, expected_volume, case, "volume");
    } else {
        assert!(
            !mesh.triangles.is_empty() && area > 0.0 && volume > 0.0,
            "{case} produced an empty or degenerate result"
        );
    }
}

fn assert_close(actual: f64, expected: f64, case: &str, property: &str) {
    let tolerance = 5.0e-4_f64.max(expected.abs() * 1.0e-8);
    assert!(
        (actual - expected).abs() <= tolerance,
        "{case} {property} mismatch: expected {expected}, got {actual}"
    );
}

fn dome_levels() -> Vec<usize> {
    let mut levels = vec![10, 100];
    if env_enabled("CSGRS_BENCH_ENABLE_STRESS") {
        levels.push(250);
    }
    if env_enabled("CSGRS_BENCH_ENABLE_DANGEROUS") {
        levels.extend([1_000, 5_000]);
    }
    levels
}

fn env_enabled(name: &str) -> bool {
    std::env::var(name).is_ok_and(|value| value == "1")
}
