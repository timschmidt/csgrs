use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    ActiveSetQuadraticRegenerationConfig, AlgebraicRootArithmeticOp,
    AlgebraicRootRefinementComparisonConfig, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    BezierPowerBasisSubstitutionConfig, Constraint, CurveIntersectionResultantConfig,
    CurveResultantParameter, EvaluationContext, Expr, IsolatedRootInterval, PolynomialCurvePoint2,
    PolynomialParametricCurve2, Problem, ProposalEngineKind, SketchSolveProblem, SolverConfig,
    SolverState, SparseResidualTerm, SymbolId, certify_candidate,
    compare_algebraic_root_representations_by_difference, context_from_problem,
    diagnose_sketch_failed_constraints, regenerate_active_set_quadratic_candidates,
    resultant_parametric_curve_intersection, search_failed_constraint_minimal_removals,
    solve_damped_least_squares, solve_dense_linear_system_bareiss_multi_rhs,
    solve_sparse_linear_system_bareiss_pattern_preserving, substitute_bezier_power_basis,
    transform_algebraic_root_mobius, transform_algebraic_root_polynomial_image,
    transform_algebraic_root_rational_image, transform_algebraic_roots_binary,
};
use std::hint::black_box;
use std::time::Duration;

#[path = "support/benchmark_report.rs"]
mod benchmark_report;
#[path = "support/retained_fuzz.rs"]
mod retained_fuzz;

const CONFIG: retained_fuzz::Config = retained_fuzz::Config {
    crate_title: "Hypersolve",
    bench_target: "retained_fuzz",
    skip_env: "HYPERSOLVE_SKIP_BENCHMARK_REPORTS",
    case_count_env: "HYPERSOLVE_RETAINED_FUZZ_CASES",
};
const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

fn mix(seed: u64, lane: u64) -> u64 {
    let mut value = seed.wrapping_add(lane.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn bounded(seed: u64, lane: u64, modulus: u64) -> i64 {
    i64::try_from(mix(seed, lane) % modulus).expect("bounded fuzz word fits i64")
}

fn signed(seed: u64, lane: u64, modulus: u64) -> i64 {
    bounded(seed, lane, modulus * 2 + 1) - i64::try_from(modulus).unwrap()
}

fn real(value: i64) -> Real {
    Real::from(value)
}

fn represented_rational_root(root: i64, symbol: u32) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: symbol as usize,
        symbol: SymbolId(symbol),
        interval_index: 0,
        polynomial_coefficients: vec![real(-root), Real::one()],
        interval: IsolatedRootInterval {
            lower: real(root),
            upper: real(root),
            exact_root: Some(real(root)),
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fn sketch_case(target: &str, seed: u64) {
    let scale = bounded(seed, 0, 15) + 1;
    let ax = signed(seed, 1, 64);
    let ay = signed(seed, 2, 64);
    let mut sketch = SketchSolveProblem::new();

    if target == "sketch_failed_constraints" {
        let a = sketch.add_point2d("a", real(ax), real(ay));
        let b = sketch.add_point2d("b", real(ax + 3 * scale), real(ay + 4 * scale));
        let distance = sketch.add_distance("distance", real(5 * scale + 1));
        sketch.add_point_point_distance("generated mismatch", a, b, distance);
        black_box(diagnose_sketch_failed_constraints(&sketch));
        return;
    }

    if target.starts_with("sketch_projected_") {
        let origin = sketch.add_point3d("origin", Real::zero(), Real::zero(), Real::zero());
        let normal = sketch.add_normal3d(
            "normal",
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
        );
        let workplane = sketch.add_workplane("workplane", origin, normal);
        let a = sketch.add_point3d("a", real(ax), real(ay), real(signed(seed, 3, 32)));
        let b = sketch.add_point3d("b", real(ax + scale), real(ay), real(signed(seed, 4, 32)));
        let distance = sketch.add_distance("distance", real(scale));
        sketch.add_projected_point_point_distance("projected", workplane, a, b, distance);
    } else {
        let a = sketch.add_point2d("a", real(ax), real(ay));
        let b = sketch.add_point2d("b", real(ax + 3 * scale), real(ay + 4 * scale));
        let distance = sketch.add_distance("distance", real(5 * scale));
        sketch.add_point_point_distance("distance row", a, b, distance);
    }

    let lowered = sketch.lower_to_problem();
    black_box(certify_candidate(
        &lowered.problem.analyze(),
        &context_from_problem(&lowered.problem),
    ));
}

fn algebraic_case(target: &str, seed: u64) {
    let root = signed(seed, 0, 24);
    let left = represented_rational_root(root, 0);
    match target {
        "algebraic_mobius" => {
            let a = signed(seed, 1, 8);
            let b = signed(seed, 2, 8);
            let c = signed(seed, 3, 8);
            let d = signed(seed, 4, 8);
            black_box(transform_algebraic_root_mobius(
                &left,
                real(a),
                real(b),
                real(c),
                real(d),
                POLICY,
            ));
        }
        "algebraic_polynomial_image" => {
            let coefficients = [
                real(signed(seed, 1, 8)),
                real(signed(seed, 2, 8)),
                real(signed(seed, 3, 8)),
            ];
            black_box(transform_algebraic_root_polynomial_image(
                &left,
                &coefficients,
                POLICY,
            ));
        }
        "algebraic_rational_image" => {
            let numerator = [real(signed(seed, 1, 8)), real(signed(seed, 2, 8))];
            let denominator = [real(signed(seed, 3, 8)), real(signed(seed, 4, 8))];
            black_box(transform_algebraic_root_rational_image(
                &left,
                &numerator,
                &denominator,
                POLICY,
            ));
        }
        "algebraic_binary" => {
            let right = represented_rational_root(signed(seed, 1, 24), 1);
            let operation = match seed % 4 {
                0 => AlgebraicRootArithmeticOp::Add,
                1 => AlgebraicRootArithmeticOp::Subtract,
                2 => AlgebraicRootArithmeticOp::Multiply,
                _ => AlgebraicRootArithmeticOp::Divide,
            };
            black_box(transform_algebraic_roots_binary(
                &left, &right, operation, POLICY,
            ));
        }
        "algebraic_difference_comparison" => {
            let right = represented_rational_root(signed(seed, 1, 24), 1);
            black_box(compare_algebraic_root_representations_by_difference(
                &left,
                &right,
                AlgebraicRootRefinementComparisonConfig {
                    max_refinement_rounds: 0,
                    ..AlgebraicRootRefinementComparisonConfig::default()
                },
            ));
        }
        unknown => panic!("unmapped algebraic fuzz target {unknown}"),
    }
}

fn dense_case(seed: u64) {
    let a = bounded(seed, 0, 15) + 1;
    let b = signed(seed, 1, 15);
    let c = bounded(seed, 2, 15) + 1;
    let first = [signed(seed, 3, 63), signed(seed, 4, 63)];
    let second = [signed(seed, 5, 63), signed(seed, 6, 63)];
    let matrix = [vec![real(a), real(b)], vec![Real::zero(), real(c)]];
    let right_hand_sides = [
        vec![real(a * first[0] + b * first[1]), real(c * first[1])],
        vec![real(a * second[0] + b * second[1]), real(c * second[1])],
    ];
    let _ = black_box(solve_dense_linear_system_bareiss_multi_rhs(
        &matrix,
        &right_hand_sides,
        -64,
        PredicatePolicy::STRICT,
    ));
}

fn sparse_case(seed: u64) {
    let a = bounded(seed, 0, 15) + 1;
    let b = signed(seed, 1, 15);
    let c = bounded(seed, 2, 15) + 1;
    let x = signed(seed, 3, 63);
    let y = signed(seed, 4, 63);
    let terms = [
        SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: real(a),
        },
        SparseResidualTerm {
            row: 0,
            column: 1,
            coefficient: real(b),
        },
        SparseResidualTerm {
            row: 1,
            column: 1,
            coefficient: real(c),
        },
    ];
    let rhs = [real(a * x + b * y), real(c * y)];
    let _ = black_box(solve_sparse_linear_system_bareiss_pattern_preserving(
        2, 2, &terms, &rhs, -64,
    ));
}

fn active_set_case(seed: u64) {
    let root = bounded(seed, 0, 31) + 1;
    let bound = signed(seed, 1, 31);
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", Real::zero());
    problem.add_constraint(Constraint::equality(
        "generated quadratic",
        x.clone() * x.clone() - Expr::int(root * root),
    ));
    let mut lower = Constraint::equality("generated lower bound", x - Expr::int(bound));
    lower.kind = hypersolve::ConstraintKind::GreaterOrEqual;
    lower.active = false;
    problem.add_constraint(lower);
    black_box(regenerate_active_set_quadratic_candidates(
        &problem.analyze(),
        &EvaluationContext::default(),
        &[true, false],
        ActiveSetQuadraticRegenerationConfig::default(),
    ));
}

fn modified_newton_case(target: &str, seed: u64) {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    if target == "modified_newton_bounded_quadratic_seed" {
        let lower = signed(seed, 0, 16);
        let upper = lower + bounded(seed, 1, 15) + 1;
        let variable = problem.add_variable("x", real(signed(seed, 2, 32)));
        problem.variables[variable.0 as usize].lower = Some(real(lower + 1));
        problem.add_constraint(Constraint::equality(
            "bounded quadratic",
            x.clone() * x.clone() - Expr::int(lower + upper) * x + Expr::int(lower * upper),
        ));
    } else {
        let y = Expr::symbol(SymbolId(1), "y");
        let offset = signed(seed, 0, 16);
        let target_y = signed(seed, 1, 16);
        let x_id = problem.add_variable("x", real(signed(seed, 2, 32)));
        problem.variables[x_id.0 as usize].upper = Some(real(target_y + offset));
        let y_id = problem.add_variable("y", real(signed(seed, 3, 32)));
        problem.variables[y_id.0 as usize].lower = Some(real(target_y));
        problem.add_constraint(Constraint::equality(
            "bounded substitution",
            x - y - Expr::int(offset),
        ));
    }
    black_box(solve_damped_least_squares(SolverState {
        problem,
        config: SolverConfig {
            max_iterations: 1,
            proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
            ..SolverConfig::default()
        },
    }));
}

fn failed_removal_case(seed: u64) {
    let value = signed(seed, 0, 32);
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(value));
    for lane in 1..=3 {
        let target = value + bounded(seed, lane, 7) + 1;
        problem.add_constraint(Constraint::equality(
            format!("generated miss {lane}"),
            x.clone() - Expr::int(target),
        ));
    }
    black_box(search_failed_constraint_minimal_removals(
        &problem.analyze(),
        &context_from_problem(&problem),
        3,
    ));
}

fn curve_case(target: &str, seed: u64) {
    let x0 = signed(seed, 0, 31);
    let y0 = signed(seed, 1, 31);
    let x1 = signed(seed, 2, 31);
    let y1 = signed(seed, 3, 31);
    if target == "curve_substitution" {
        let points = [
            PolynomialCurvePoint2::new(real(x0), real(y0)),
            PolynomialCurvePoint2::new(real(x1), real(y1)),
        ];
        black_box(substitute_bezier_power_basis(
            &points,
            BezierPowerBasisSubstitutionConfig::default(),
        ));
    } else {
        let height = bounded(seed, 4, 31) + 1;
        let parabola = PolynomialParametricCurve2::new(
            vec![real(x0), Real::one()],
            vec![Real::zero(), Real::zero(), Real::one()],
        );
        let horizontal =
            PolynomialParametricCurve2::new(vec![real(x0), Real::one()], vec![real(height)]);
        black_box(resultant_parametric_curve_intersection(
            &parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        ));
    }
}

fn representation_case(seed: u64) {
    let value = match seed % 3 {
        0 => Real::pi() * real(bounded(seed, 0, 7) + 1),
        1 => real(bounded(seed, 0, 31) + 1)
            .sqrt()
            .expect("positive fuzz radicand"),
        _ => real(signed(seed, 0, 64)),
    };
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", value.clone());
    problem.add_constraint(Constraint::equality("identity", x - Expr::real(value)));
    black_box(certify_candidate(
        &problem.analyze(),
        &context_from_problem(&problem),
    ));
}

fn run_case(target: &str, seed: u64) {
    match target {
        target if target.starts_with("sketch_") => sketch_case(target, seed),
        target if target.starts_with("algebraic_") => algebraic_case(target, seed),
        "dense_bareiss_multi_rhs" => dense_case(seed),
        "sparse_pattern_preserving_bareiss" => sparse_case(seed),
        "active_set_quadratic_regeneration" => active_set_case(seed),
        "modified_newton_bounded_quadratic_seed" | "modified_newton_bounded_substitution_seed" => {
            modified_newton_case(target, seed)
        }
        "failed_constraint_minimal_removals" => failed_removal_case(seed),
        "curve_resultant" | "curve_substitution" => curve_case(target, seed),
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
