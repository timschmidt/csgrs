use std::hint::black_box;
use std::time::Instant;

use hyperreal::Real;
use hypersolve::{
    AlgebraicFiberRootCountStatus, AlgebraicRootKind, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, BivariatePolynomial,
    CurveResultantParameter, IsolatedRootInterval, PredicatePolicy, SymbolId,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter,
};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: i64) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero benchmark denominator")
}

fn main() {
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
        kind: AlgebraicRootKind::IsolatingInterval,
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
    println!(
        "algebraic_fiber_even_multiplicity: {iterations} iterations in {elapsed:?} ({:?}/iter), root_checksum={root_count}, refinement_checksum={refinement_steps}",
        elapsed / iterations
    );

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
    println!(
        "algebraic_common_fiber_degree_drop: {iterations} iterations in {elapsed:?} ({:?}/iter), root_checksum={common_root_count}, refinement_checksum={common_refinement_steps}",
        elapsed / iterations
    );
}
