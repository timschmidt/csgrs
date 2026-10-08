use criterion::{Criterion, criterion_group, criterion_main};
use hyperlattice::{Complex, Matrix3, Rational, Real, Vector3};
use std::hint::black_box;
use std::time::Duration;

#[path = "support/benchmark_report.rs"]
mod benchmark_report;
#[path = "support/retained_fuzz.rs"]
mod retained_fuzz;

const CONFIG: retained_fuzz::Config = retained_fuzz::Config {
    crate_title: "Hyperlattice",
    bench_target: "retained_fuzz",
    skip_env: "HYPERLATTICE_SKIP_BENCHMARK_REPORTS",
    case_count_env: "HYPERLATTICE_RETAINED_FUZZ_CASES",
};

fn mix(seed: u64, lane: u64) -> u64 {
    let mut value = seed.wrapping_add(lane.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn real(seed: u64, lane: u64) -> Real {
    let bits = mix(seed, lane);
    let magnitude = i64::try_from(bits % 2_001).expect("bounded magnitude fits i64") + 1;
    let numerator = if bits & 1 == 0 { magnitude } else { -magnitude };
    let denominator = (bits.rotate_left(17) % 257) + 1;
    Real::from(Rational::fraction(numerator, denominator).expect("positive denominator"))
}

fn run_case(target: &str, seed: u64) {
    match target {
        "scalar_ops" => {
            let a = real(seed, 0);
            let b = real(seed, 1);
            let value = (&a * &b + &a - &b).powi_i64(i64::try_from(seed % 5 + 1).unwrap());
            let _ = black_box(value);
        }
        "complex_ops" => {
            let left = Complex::new(real(seed, 0), real(seed, 1));
            let right = Complex::new(real(seed, 2), real(seed, 3));
            let product = &left * &right;
            black_box(product.norm_squared());
            let _ = black_box(left.powi(i64::try_from(seed % 5 + 1).unwrap()));
        }
        "vector_ops" => {
            let left = Vector3::new([real(seed, 0), real(seed, 1), real(seed, 2)]);
            let right = Vector3::new([real(seed, 3), real(seed, 4), real(seed, 5)]);
            black_box(left.dot(&right));
            black_box(left.cross(&right));
            black_box(left.norm_squared());
        }
        "matrix_ops" => {
            let matrix = Matrix3::new([
                [real(seed, 0), real(seed, 1), Real::zero()],
                [Real::zero(), real(seed, 2), real(seed, 3)],
                [Real::zero(), Real::zero(), real(seed, 4)],
            ]);
            black_box(matrix.determinant());
            let _ = black_box(matrix.inverse_checked());
        }
        "hyperreal_representations" => {
            let rational = real(seed, 0);
            let symbolic = if seed & 1 == 0 {
                Real::pi() * rational
            } else {
                (rational.clone() * rational + Real::one())
                    .sqrt()
                    .expect("positive fuzz radicand")
            };
            black_box(symbolic.detailed_facts());
            black_box(symbolic.certified_dyadic_interval(-64));
        }
        unknown => panic!("unmapped fuzz target {unknown}"),
    }
}

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
