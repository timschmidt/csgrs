use std::hint::black_box;
use std::time::Instant;

use hyperreal::Real;
use hypersolve::{
    AlgebraicFiberRootCountStatus, AlgebraicRootKind, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, BivariatePolynomial,
    BivariatePolynomialComponentStatus, CurveIntersectionResultantConfig, CurveResultantParameter,
    IsolatedRootInterval, PredicatePolicy, SymbolId,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter,
    parameter_component_bivariate_polynomial_system,
};

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
    println!(
        "rational_quadratic_common_fiber_two_components: {iterations} iterations in {elapsed:?} ({:?}/iter), component_checksum={component_checksum}",
        elapsed / iterations
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
    println!(
        "implicit_quadratic_common_fiber: {iterations} iterations in {elapsed:?} ({:?}/iter), component_checksum={component_checksum}",
        elapsed / iterations
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
    println!(
        "implicit_quadratic_high_cofactor: {iterations} iterations in {elapsed:?} ({:?}/iter), component_checksum={component_checksum}",
        elapsed / iterations
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
    println!(
        "rational_repeated_cubic_common_fiber_three_components: {iterations} iterations in {elapsed:?} ({:?}/iter), component_checksum={component_checksum}",
        elapsed / iterations
    );
}
