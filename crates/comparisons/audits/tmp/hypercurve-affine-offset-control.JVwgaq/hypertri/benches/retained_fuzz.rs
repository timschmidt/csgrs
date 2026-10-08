use criterion::{Criterion, criterion_group, criterion_main};
use hypertri::{
    Constraint, Point2, PointD, PolygonInput, PredicatePolicy, Rational, Real,
    TriangulationContext, TriangulationOptions,
};
use std::hint::black_box;
use std::time::Duration;

#[path = "support/benchmark_report.rs"]
mod benchmark_report;
#[path = "support/retained_fuzz.rs"]
mod retained_fuzz;

const CONFIG: retained_fuzz::Config = retained_fuzz::Config {
    crate_title: "Hypertri",
    bench_target: "retained_fuzz",
    skip_env: "HYPERTRI_SKIP_BENCHMARK_REPORTS",
    case_count_env: "HYPERTRI_RETAINED_FUZZ_CASES",
};
const CONTEXT: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn mix(seed: u64, lane: u64) -> u64 {
    let mut value = seed.wrapping_add(lane.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).expect("positive denominator"))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn topology_case(seed: u64) {
    let x = i64::try_from(mix(seed, 0) % 101).unwrap() - 50;
    let y = i64::try_from(mix(seed, 1) % 101).unwrap() - 50;
    let width = i64::try_from(mix(seed, 2) % 29).unwrap() + 3;
    let height = i64::try_from(mix(seed, 3) % 29).unwrap() + 3;
    match seed % 4 {
        0 => {
            let notch_x =
                i64::try_from(mix(seed, 4) % u64::try_from(width - 1).unwrap()).unwrap() + 1;
            let notch_y =
                i64::try_from(mix(seed, 5) % u64::try_from(height - 1).unwrap()).unwrap() + 1;
            let vertices = vec![
                p(x, y),
                p(x + width, y),
                p(x + width, y + height),
                p(x + notch_x, y + height),
                p(x + notch_x, y + notch_y),
                p(x, y + notch_y),
            ];
            let _ = black_box(hypertri::earcut(&CONTEXT, &vertices, &[]));
        }
        1 => {
            let denominator = mix(seed, 4) % 31 + 1;
            let points = vec![
                p(x, y),
                Point2::new(Real::from(x + width), q(y + height, denominator)),
                Point2::new(Real::from(x), q(y + height, denominator)),
                p(x + width, y),
            ];
            let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];
            let _ = black_box(hypertri::cdt::constrained_delaunay(
                &CONTEXT,
                &points,
                &constraints,
            ));
        }
        2 => {
            let points = vec![
                p(x, y),
                p(x + width, y),
                p(x + width, y + height),
                p(x, y + height),
                p(x + width / 2, y + height / 2),
            ];
            let _ = black_box(hypertri::cdt::delaunay(&CONTEXT, &points));
        }
        _ => {
            let points = vec![
                PointD::new(vec![Real::from(x), Real::from(y), Real::zero()]),
                PointD::new(vec![Real::from(x + width), Real::from(y), Real::zero()]),
                PointD::new(vec![Real::from(x), Real::from(y + height), Real::zero()]),
                PointD::new(vec![Real::from(x), Real::from(y), Real::from(width)]),
                PointD::new(vec![
                    q(x * 4 + width, 4),
                    q(y * 4 + height, 4),
                    q(width + height, 8),
                ]),
            ];
            let _ = black_box(hypertri::nd::delaunay_complex(&CONTEXT, &points));
        }
    }
}

fn representation_case(seed: u64) {
    let width = i64::try_from(mix(seed, 0) % 19).unwrap() + 3;
    let height = i64::try_from(mix(seed, 1) % 19).unwrap() + 3;
    let tx = match seed % 3 {
        0 => q(
            i64::try_from(mix(seed, 2) % 101).unwrap() - 50,
            mix(seed, 3) % 31 + 1,
        ),
        1 => Real::pi() * q(i64::try_from(mix(seed, 2) % 11).unwrap() + 1, 7),
        _ => Real::from(i64::try_from(mix(seed, 2) % 31).unwrap() + 1)
            .sqrt()
            .expect("positive fuzz radicand"),
    };
    let ty = &tx + q(i64::try_from(mix(seed, 4) % 17).unwrap() + 1, 11);
    let points = vec![
        Point2::new(tx.clone(), ty.clone()),
        Point2::new(&tx + Real::from(width), ty.clone()),
        Point2::new(&tx + Real::from(width), &ty + Real::from(height)),
        Point2::new(tx.clone(), &ty + Real::from(height)),
    ];
    match seed % 3 {
        0 => {
            let _ = black_box(hypertri::earcut(&CONTEXT, &points, &[]));
        }
        1 => {
            let _ = black_box(hypertri::cdt::delaunay_spatial(&CONTEXT, &points));
        }
        _ => {
            let input = PolygonInput::new(points, Vec::new());
            let _ = black_box(hypertri::triangulate_polygon(
                &CONTEXT,
                &input,
                TriangulationOptions::default(),
            ));
        }
    }
}

fn run_case(target: &str, seed: u64) {
    match target {
        "topology_invariants" => topology_case(seed),
        "hyperreal_representations" => representation_case(seed),
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
