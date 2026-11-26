//! Opt-in timing of pinned Solidean request graphs with projected mass-property checks.
//!
//! `SOLIDEAN_BENCH` enables acquisition/execution. Loading is outside each timer;
//! request execution includes exact ingress from parsed binary64 coordinates,
//! intermediate destruction and optional diagnostics. Validation, count checksums
//! and final-output destruction are outside the reported interval. Successful rows
//! do not establish source fidelity, exact set equality or topology independently.

/// Shared selection and caller-supplied-duration CSV harness.
#[path = "../support/harness.rs"]
#[allow(dead_code)]
mod support;

/// Pinned fixture acquisition, restricted request interpreter and projected measures.
#[path = "../support/solidean.rs"]
mod solidean;

use std::hint::black_box;
use std::time::Instant;

use csgrs::TriangleMesh;
use solidean::Request;
use support::{Config, Measurement, print_header};

/// Suite label used by shared filters, stress sampling and emitted CSV rows.
const SUITE: &str = "published-solidean";

/// Prints the header, checks opt-in, then selects three fixed requests and dome levels.
/// Expected area/volume pairs are retained decimal benchmark references with
/// tolerance checks, not exact geometric oracles. Any loader/validation panic aborts.
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

/// Loads a selected request once, then times execution and validates every repetition.
/// Unselected cases never call `load`. Only `execute` lies inside each returned
/// duration; warmups also execute validation. Requests/caches are reused. Operation
/// and input-triangle totals are printed after timing; no certainty receipt is emitted.
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

/// Requires finite projected area/absolute volume and checks supplied reference values.
/// Without references, requires faces and positive measures. With `(0, 0)`, a
/// tolerance-sized nonempty result can pass; this is not a canonical-empty assertion.
/// Neither branch establishes closedness, winding or exact represented-set equality.
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

/// Asserts absolute error at most `max(5e-4, abs(expected) * 1e-8)`.
/// Current callers supply finite reference values and separately check finite actuals.
fn assert_close(actual: f64, expected: f64, case: &str, property: &str) {
    let tolerance = 5.0e-4_f64.max(expected.abs() * 1.0e-8);
    assert!(
        (actual - expected).abs() <= tolerance,
        "{case} {property} mismatch: expected {expected}, got {actual}"
    );
}

/// Selects 10/100 by default, 250 for stress and 1000/5000 for dangerous mode.
/// Both gates require exactly `"1"`; dangerous mode alone does not add level 250.
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

/// Recognizes exactly `"1"`; missing, non-Unicode and other values disable the gate.
fn env_enabled(name: &str) -> bool {
    std::env::var(name).is_ok_and(|value| value == "1")
}
