use std::fs;
use std::hint::black_box;
use std::time::{Duration, Instant};

use hyperreal::Real;
use hypersolve::{
    AlgebraicFiberPolynomialImageProjectionConfig, AlgebraicFiberPolynomialImageProjectionStatus,
    AlgebraicFiberRootCountStatus, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, BivariatePolynomial, BivariatePolynomialComponentStatus,
    CurveIntersectionResultantConfig, CurveResultantParameter, IsolatedRootInterval,
    PredicatePolicy, SymbolId, count_bivariate_common_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter_intervals,
    parameter_component_bivariate_polynomial_system, project_algebraic_fiber_polynomial_image,
};

#[path = "support/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

struct TimingRow {
    name: &'static str,
    iterations: u32,
    elapsed: Duration,
    detail: String,
}

fn record_timing(
    rows: &mut Vec<TimingRow>,
    name: &'static str,
    iterations: u32,
    elapsed: Duration,
    detail: String,
) {
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), {detail}",
        elapsed / iterations,
    );
    rows.push(TimingRow {
        name,
        iterations,
        elapsed,
        detail,
    });
}

fn write_timing_report(rows: &[TimingRow]) {
    if std::env::var_os("HYPERSOLVE_SKIP_BENCHMARK_REPORTS").is_some() {
        return;
    }
    let mut report = String::from(
        "# Hypersolve Algebraic Fiber Benchmarks\n\n\
Generated automatically by `cargo bench --bench algebraic_fiber`. These are deterministic wall-clock throughput probes with exact result checks, not Criterion statistical estimates. Override the default iteration count with `HYPERSOLVE_FIBER_BENCH_ITERATIONS`.\n\n\
| Benchmark | Iterations | Total | Mean per iteration | Validation checksum |\n\
| --- | ---: | ---: | ---: | --- |\n",
    );
    for row in rows {
        let mean_ns = row.elapsed.as_secs_f64() * 1_000_000_000.0 / f64::from(row.iterations);
        report.push_str(&format!(
            "| `{}` | {} | {:.3} s | {} | {} |\n",
            row.name,
            row.iterations,
            row.elapsed.as_secs_f64(),
            format_duration(mean_ns),
            row.detail,
        ));
    }
    fs::write("algebraic_fiber_benchmarks.md", report)
        .expect("algebraic fiber benchmark report should be writable");
}

fn format_duration(ns: f64) -> String {
    if ns < 1_000.0 {
        format!("{ns:.2} ns")
    } else if ns < 1_000_000.0 {
        format!("{:.2} us", ns / 1_000.0)
    } else if ns < 1_000_000_000.0 {
        format!("{:.2} ms", ns / 1_000_000.0)
    } else {
        format!("{:.2} s", ns / 1_000_000_000.0)
    }
}

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: i64) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero benchmark denominator")
}

fn multiply_bivariate(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    let first_columns = first.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let second_columns = second.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let mut coefficients = vec![
        vec![Real::zero(); first_columns + second_columns - 1];
        first.coefficients.len() + second.coefficients.len() - 1
    ];
    for (first_power, first_row) in first.coefficients.iter().enumerate() {
        for (second_power, second_row) in second.coefficients.iter().enumerate() {
            for (first_column, first_coefficient) in first_row.iter().enumerate() {
                for (second_column, second_coefficient) in second_row.iter().enumerate() {
                    coefficients[first_power + second_power][first_column + second_column] +=
                        first_coefficient * second_coefficient;
                }
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn main() {
    let mut timing_rows = Vec::new();
    let retained_root = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![r(-1), r(0), r(0), r(2)],
        interval: IsolatedRootInterval {
            lower: q(3, 4),
            upper: q(4, 5),
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    let fiber = BivariatePolynomial::new(vec![
        vec![r(0), r(0), r(1)],
        vec![],
        vec![r(0), r(-2)],
        vec![],
        vec![r(1)],
    ]);
    let lower = q(3, 5);
    let upper = q(2, 3);
    let iterations = std::env::var("HYPERSOLVE_FIBER_BENCH_ITERATIONS")
        .ok()
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(1_000);
    let started = Instant::now();
    let mut root_count = 0_usize;
    let mut refinement_steps = 0_usize;
    for _ in 0..iterations {
        let report = count_bivariate_fiber_roots_at_algebraic_parameter(
            black_box(&fiber),
            CurveResultantParameter::First,
            black_box(&retained_root),
            black_box(&lower),
            black_box(&upper),
            PredicatePolicy::STRICT,
        );
        assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
        root_count += black_box(report.distinct_root_count.unwrap_or(0));
        refinement_steps += black_box(report.retained_refinement_steps);
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "algebraic_fiber_even_multiplicity",
        iterations,
        elapsed,
        format!("root={root_count}; refinement={refinement_steps}"),
    );

    let batch_interval_values = [
        (q(0, 1), q(1, 2)),
        (q(3, 5), q(2, 3)),
        (q(2, 3), q(1, 1)),
        (q(1, 10), q(11, 20)),
        (q(31, 50), q(13, 20)),
        (q(7, 10), q(9, 10)),
        (q(29, 50), q(33, 50)),
        (q(1, 4), q(3, 4)),
    ];
    let batch_intervals = batch_interval_values
        .iter()
        .map(|(lower, upper)| (lower, upper))
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut independent_checksum = 0_usize;
    for _ in 0..iterations {
        for (lower, upper) in &batch_intervals {
            let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                black_box(&fiber),
                CurveResultantParameter::First,
                black_box(&retained_root),
                black_box(lower),
                black_box(upper),
                PredicatePolicy::STRICT,
            );
            assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
            independent_checksum += black_box(report.distinct_root_count.unwrap_or(0));
        }
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "algebraic_fiber_eight_independent_intervals",
        iterations,
        elapsed,
        format!("root={independent_checksum}"),
    );

    let started = Instant::now();
    let mut batch_checksum = 0_usize;
    for _ in 0..iterations {
        let reports = count_bivariate_fiber_roots_at_algebraic_parameter_intervals(
            black_box(&fiber),
            CurveResultantParameter::First,
            black_box(&retained_root),
            black_box(&batch_intervals),
            PredicatePolicy::STRICT,
        );
        assert_eq!(reports.len(), batch_intervals.len());
        for report in reports {
            assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
            batch_checksum += black_box(report.distinct_root_count.unwrap_or(0));
        }
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "algebraic_fiber_eight_batched_intervals",
        iterations,
        elapsed,
        format!("root={batch_checksum}"),
    );
    assert_eq!(batch_checksum, independent_checksum);

    let second = BivariatePolynomial::new(vec![
        vec![r(1), r(2), r(0), r(-4)],
        vec![],
        vec![r(-2)],
        vec![r(-2), r(0), r(0), r(8)],
    ]);
    let first = BivariatePolynomial::new(vec![
        vec![r(1), r(1), r(0), r(-4)],
        vec![],
        vec![r(-1)],
        vec![r(-2), r(0), r(0), r(8)],
    ]);
    let started = Instant::now();
    let mut common_root_count = 0_usize;
    let mut common_refinement_steps = 0_usize;
    for _ in 0..iterations {
        let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
            black_box(&first),
            black_box(&second),
            CurveResultantParameter::First,
            black_box(&retained_root),
            black_box(&lower),
            black_box(&upper),
            PredicatePolicy::STRICT,
        );
        assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
        common_root_count += black_box(report.distinct_root_count.unwrap_or(0));
        common_refinement_steps += black_box(report.retained_refinement_steps);
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "algebraic_common_fiber_degree_drop",
        iterations,
        elapsed,
        format!("root={common_root_count}; refinement={common_refinement_steps}"),
    );

    let sqrt_two = r(2).sqrt().expect("positive square root");
    let sqrt_three = r(3).sqrt().expect("positive square root");
    let radical_sum = &sqrt_two + &sqrt_three;
    let radical_zero =
        &radical_sum * &radical_sum - (r(5) + r(2) * r(6).sqrt().expect("positive square root"));
    let linear_fiber = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![r(-1)]]);
    let image_relation = BivariatePolynomial::new(vec![
        vec![Real::one() + radical_zero, Real::one()],
        vec![Real::one(), Real::one()],
    ]);
    let image_config = AlgebraicFiberPolynomialImageProjectionConfig {
        max_fiber_degree: 1,
        max_retained_degree: 3,
        max_image_degree_bound: 1,
    };
    let started = Instant::now();
    let mut image_checksum = 0_usize;
    for _ in 0..iterations {
        let report = project_algebraic_fiber_polynomial_image(
            black_box(&linear_fiber),
            CurveResultantParameter::First,
            black_box(&image_relation),
            CurveResultantParameter::Second,
            black_box(&retained_root),
            image_config,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicFiberPolynomialImageProjectionStatus::Constructed
        );
        assert_eq!(report.coefficients, vec![Real::one(), Real::one()]);
        image_checksum += black_box(report.coefficients.len());
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "algebraic_image_policy_zero_content",
        iterations,
        elapsed,
        format!("coefficient={image_checksum}"),
    );

    let first_component = BivariatePolynomial::new(vec![vec![r(0), r(1)], vec![r(-1)]]);
    let second_component = BivariatePolynomial::new(vec![vec![r(-1), r(1)], vec![r(1)]]);
    let common = multiply_bivariate(&first_component, &second_component);
    let first = multiply_bivariate(
        &common,
        &BivariatePolynomial::new(vec![vec![r(1), r(1)], vec![r(1)]]),
    );
    let second = multiply_bivariate(
        &common,
        &BivariatePolynomial::new(vec![vec![r(2), r(1)], vec![r(-1)]]),
    );
    let config = CurveIntersectionResultantConfig {
        min_precision: -512,
        max_resultant_degree: 64,
    };
    let started = Instant::now();
    let mut component_checksum = 0_usize;
    for _ in 0..iterations {
        let first_report = parameter_component_bivariate_polynomial_system(
            black_box(&first),
            black_box(&second),
            CurveResultantParameter::First,
            config,
        );
        assert_eq!(
            first_report.status,
            BivariatePolynomialComponentStatus::Rational
        );
        let reduced = first_report
            .reduced_equations
            .as_ref()
            .expect("the first factor retains its exact residual");
        let second_report = parameter_component_bivariate_polynomial_system(
            black_box(&reduced[0]),
            black_box(&reduced[1]),
            CurveResultantParameter::First,
            config,
        );
        assert_eq!(
            second_report.status,
            BivariatePolynomialComponentStatus::Rational
        );
        component_checksum += black_box(
            first_report.numerator_coefficients.len() + second_report.numerator_coefficients.len(),
        );
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "rational_quadratic_common_fiber_two_components",
        iterations,
        elapsed,
        format!("component={component_checksum}"),
    );

    let implicit_component =
        BivariatePolynomial::new(vec![vec![r(-1), r(0), r(1)], vec![r(0), r(-1)]]);
    let first = multiply_bivariate(
        &implicit_component,
        &BivariatePolynomial::new(vec![vec![r(1), r(1)], vec![r(1)]]),
    );
    let second = multiply_bivariate(
        &implicit_component,
        &BivariatePolynomial::new(vec![vec![r(2), r(2)], vec![r(-1)]]),
    );
    let started = Instant::now();
    let mut component_checksum = 0_usize;
    for _ in 0..iterations {
        let report = parameter_component_bivariate_polynomial_system(
            black_box(&first),
            black_box(&second),
            CurveResultantParameter::First,
            config,
        );
        assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
        component_checksum += black_box(
            report
                .implicit_component
                .as_ref()
                .expect("the irreducible quadratic remains explicit")
                .coefficients
                .iter()
                .map(Vec::len)
                .sum::<usize>(),
        );
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "implicit_quadratic_common_fiber",
        iterations,
        elapsed,
        format!("component={component_checksum}"),
    );

    let implicit_component = BivariatePolynomial::new(vec![
        vec![r(0), r(-8), r(2)],
        vec![r(6), r(-3)],
        vec![r(6), r(-3)],
    ]);
    let first = multiply_bivariate(
        &implicit_component,
        &BivariatePolynomial::new(vec![
            vec![r(1), r(0), r(0), r(0), r(1)],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![r(1)],
        ]),
    );
    let second = multiply_bivariate(
        &implicit_component,
        &BivariatePolynomial::new(vec![
            vec![
                r(2),
                r(0),
                r(0),
                r(0),
                r(0),
                r(0),
                r(0),
                r(0),
                r(0),
                r(0),
                r(1),
            ],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![],
            vec![r(1)],
        ]),
    );
    let config = CurveIntersectionResultantConfig {
        min_precision: -512,
        max_resultant_degree: 256,
    };
    let started = Instant::now();
    let mut component_checksum = 0_usize;
    for _ in 0..iterations {
        let report = parameter_component_bivariate_polynomial_system(
            black_box(&first),
            black_box(&second),
            CurveResultantParameter::Second,
            config,
        );
        assert_eq!(report.status, BivariatePolynomialComponentStatus::Implicit);
        component_checksum += black_box(
            report
                .implicit_component
                .as_ref()
                .expect("the high-cofactor component remains explicit")
                .coefficients
                .iter()
                .map(Vec::len)
                .sum::<usize>(),
        );
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "implicit_quadratic_high_cofactor",
        iterations,
        elapsed,
        format!("component={component_checksum}"),
    );

    let repeated_component = BivariatePolynomial::new(vec![vec![r(0), r(1)], vec![r(-1)]]);
    let distinct_component = BivariatePolynomial::new(vec![vec![r(-1), r(1)], vec![r(1)]]);
    let repeated_common = multiply_bivariate(
        &multiply_bivariate(&repeated_component, &repeated_component),
        &distinct_component,
    );
    let first = multiply_bivariate(
        &repeated_common,
        &BivariatePolynomial::new(vec![vec![r(1), r(1)], vec![r(1)]]),
    );
    let second = multiply_bivariate(
        &repeated_common,
        &BivariatePolynomial::new(vec![vec![r(2), r(1)], vec![r(-1)]]),
    );
    let started = Instant::now();
    let mut component_checksum = 0_usize;
    for _ in 0..iterations {
        let mut residual = [first.clone(), second.clone()];
        for _ in 0..3 {
            let report = parameter_component_bivariate_polynomial_system(
                black_box(&residual[0]),
                black_box(&residual[1]),
                CurveResultantParameter::First,
                config,
            );
            assert_eq!(report.status, BivariatePolynomialComponentStatus::Rational);
            component_checksum += black_box(report.numerator_coefficients.len());
            residual = report
                .reduced_equations
                .expect("each cubic factor retains its exact residual");
        }
    }
    let elapsed = started.elapsed();
    record_timing(
        &mut timing_rows,
        "rational_repeated_cubic_common_fiber_three_components",
        iterations,
        elapsed,
        format!("component={component_checksum}"),
    );

    write_timing_report(&timing_rows);
    if !benchmark_report::reports_disabled() {
        match benchmark_report::write_benchmarks_md() {
            Ok(summary) => eprintln!(
                "updated {} from {} Criterion rows and {} benchmark suites",
                summary.path.display(),
                summary.rows,
                summary.suites,
            ),
            Err(error) => eprintln!("failed to update benchmarks.md: {error}"),
        }
    }
}
