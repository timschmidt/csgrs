//! Criterion workloads for generated seeds associated with the two Hyperlimit fuzz target names.
//!
//! The generators are defined here; they do not replay libFuzzer corpus bytes or
//! invoke fuzz-target invariant checks. APPROXIMATE_512 outcomes are black-boxed,
//! so elapsed times alone do not establish certified geometric completion.
//!
//! Criterion 0.5 generates two harness functions below. The public benches function
//! creates a default Criterion instance, configures it from argv, calls
//! bench_retained_fuzz, then the shared report callback. Generated main invokes that
//! group and a separate default Criterion instance's final_summary after argument
//! configuration. The macros do not attach item Rustdoc to those generated functions;
//! their behavior is described here rather than claimed as per-item doc coverage.

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::{
    Point2, Point3, PredicatePolicy, classify_real_sign, compare_reals, incircle2, insphere3,
    orient2, orient3,
};
use hyperreal::{Rational, Real};
use std::hint::black_box;
use std::time::Duration;

/// Shared stored-Criterion report writer, invoked after retained-case benchmark registration.
mod benchmark_report;
/// Shared seed timing, historical retention, promotion, and best-effort report support.
#[path = "support/retained_fuzz.rs"]
mod retained_fuzz;

/// Crate-local report names and environment controls for retained-case selection.
const CONFIG: retained_fuzz::Config = retained_fuzz::Config {
    crate_title: "Hyperlimit",
    bench_target: "retained_fuzz",
    skip_env: "HYPERLIMIT_SKIP_BENCHMARK_REPORTS",
    case_count_env: "HYPERLIMIT_RETAINED_FUZZ_CASES",
};
/// Approximation-permitting policy used by all generated predicate probes, without outcome assertions.
const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

/// Mix seed and lane with wrapping u64 arithmetic to obtain reproducible generator bits.
fn mix(seed: u64, lane: u64) -> u64 {
    let mut value = seed.wrapping_add(lane.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

/// Construct a nonzero rational with signed numerator magnitude 1..=2001 and denominator 1..=1009.
///
/// Both are deterministically derived from the same mixed bits. Positive denominator
/// and bounded conversion justify the constructor expects; no float input is used.
fn real(seed: u64, lane: u64) -> Real {
    let bits = mix(seed, lane);
    let magnitude = i64::try_from(bits % 2_001).expect("bounded magnitude fits i64") + 1;
    let numerator = if bits & 1 == 0 { magnitude } else { -magnitude };
    let denominator = (bits.rotate_left(19) % 1_009) + 1;
    Real::from(Rational::fraction(numerator, denominator).expect("positive denominator"))
}

/// Build two rational coordinates from adjacent lanes.
///
/// The caller must keep lane+1 within u64 to avoid ordinary addition overflow.
fn point2(seed: u64, lane: u64) -> Point2 {
    Point2::new(real(seed, lane), real(seed, lane + 1))
}

/// Build three rational coordinates from adjacent lanes.
///
/// The caller must keep lane+2 within u64; benchmark call sites use small fixed lanes.
fn point3(seed: u64, lane: u64) -> Point3 {
    Point3::new(real(seed, lane), real(seed, lane + 1), real(seed, lane + 2))
}

/// Construct and execute the generated workload selected by one of two manifest target names.
///
/// predicate_invariants runs planar/spatial orientation and in-circle/in-sphere on
/// nonzero rational coordinates. hyperreal_representations uses pi times a rational
/// for even seeds or sqrt(offset^2+1) for odd seeds, then tests an axis-aligned unit
/// triangle on symbolic offsets, scalar sign, and ordering. These are benchmark
/// generators, not decoders of the corresponding fuzz targets' corpus bytes.
///
/// Input construction and result destruction are part of each call. Black-boxes
/// outcomes without asserting values, certainty, or success; names do not establish
/// that invariants or all representations were tested.
///
/// # Panics
///
/// Panics on an unmapped target. Also expects construction of a square root with
/// strictly positive radicand to succeed in the odd-seed representation branch.
fn run_case(target: &str, seed: u64) {
    match target {
        "predicate_invariants" => {
            let a = point2(seed, 0);
            let b = point2(seed, 2);
            let c = point2(seed, 4);
            let d = point2(seed, 6);
            black_box(orient2(&a, &b, &c, POLICY));
            black_box(incircle2(&a, &b, &c, &d, POLICY));

            let p = point3(seed, 8);
            let q = point3(seed, 11);
            let r = point3(seed, 14);
            let s = point3(seed, 17);
            let t = point3(seed, 20);
            black_box(orient3(&p, &q, &r, &s, POLICY));
            black_box(insphere3(&p, &q, &r, &s, &t, POLICY));
        }
        "hyperreal_representations" => {
            let offset = real(seed, 0);
            let left = if seed & 1 == 0 {
                Real::pi() * offset.clone()
            } else {
                (offset.clone() * offset.clone() + Real::one())
                    .sqrt()
                    .expect("positive fuzz radicand")
            };
            let right = &left + real(seed, 1);
            let a = Point2::new(left.clone(), right.clone());
            let b = Point2::new(&left + Real::one(), right.clone());
            let c = Point2::new(left.clone(), &right + Real::one());
            black_box(orient2(&a, &b, &c, POLICY));
            black_box(classify_real_sign(&left, POLICY));
            black_box(compare_reals(&left, &right, POLICY));
        }
        unknown => panic!("unmapped fuzz target {unknown}"),
    }
}

/// Collect and rotate cases, then register individual and aggregate Criterion replay workloads.
///
/// Metadata-only invocations return before registration. Otherwise parses target
/// names from the embedded fuzz manifest and performs initial probes/refresh before
/// Criterion filters the registered rows. The report skip flag does not suppress
/// initial sampling or existing-promotion re-probes. Report writes can occur during
/// refresh before any timed Criterion iteration.
///
/// Both groups use sample size 10, 25 ms warm-up, and 100 ms requested measurement
/// time. The aggregate row name replay_promoted_100 is fixed even when promotion
/// returns fewer cases. Timed calls include generated-input construction and do not
/// assert exact completion. Existing/historical target mismatches can panic during
/// replay; support-module write failures are logged rather than propagated.
fn bench_retained_fuzz(c: &mut Criterion) {
    if retained_fuzz::metadata_only_invocation() {
        return;
    }
    let targets = retained_fuzz::fuzz_targets_from_manifest(include_str!("../fuzz/Cargo.toml"));
    let current = retained_fuzz::collect_cases(CONFIG, &targets, run_case);
    let refresh = retained_fuzz::refresh(CONFIG, &targets, &current, run_case);

    let mut group = c.benchmark_group("promoted_fuzz_worst_performers");
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(25));
    group.measurement_time(Duration::from_millis(100));
    for case in &refresh.promoted {
        let name = case.criterion_name();
        let target = case.target.clone();
        let seed = case.seed;
        group.bench_function(name, move |b| {
            b.iter(|| run_case(black_box(&target), black_box(seed)))
        });
    }
    group.finish();

    let promoted = refresh.promoted;
    let mut score = c.benchmark_group("promoted_slow_offender_score");
    score.sample_size(10);
    score.warm_up_time(Duration::from_millis(25));
    score.measurement_time(Duration::from_millis(100));
    score.bench_function("replay_promoted_100", move |b| {
        b.iter(|| {
            for case in &promoted {
                run_case(black_box(&case.target), black_box(case.seed));
            }
        })
    });
    score.finish();
}

criterion_group!(
    benches,
    bench_retained_fuzz,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
