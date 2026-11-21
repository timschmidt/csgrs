//! Replay the original equations at the reported root after coupled elimination.
#[path = "support/represented_roots.rs"]
mod oracle;

use hyperreal::Real;
use hypersolve::{
    AffineKrawczykStatus, Constraint, EvaluationContext, Expr, PredicatePolicy, Problem, SymbolId,
    VariableBall, certify_affine_krawczyk_box, context_from_problem,
};

#[test]
fn pivot_order_preserves_all_three_original_equations() {
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let tiny = Real::from(2).powi_i64(-2048).unwrap();
    let radical = Real::from(2).sqrt().unwrap();
    for c in [tiny.clone(), -tiny, radical.clone(), -radical] {
        for order in orders {
            let mut problem = Problem::default();
            let symbols = ["x", "y", "z"]
                .map(|name| SymbolId::from(problem.add_variable(name, Real::zero())));
            let [x, y, z] = symbols.map(Expr::anonymous_symbol);
            // Triangular before permutation, with unique root (1, -c, c).
            let rows = [
                x.clone() - Expr::int(1),
                Expr::real(c.clone()) * x + y.clone(),
                y + z,
            ];
            for i in order {
                problem.add_constraint(Constraint::equality("row", rows[i].clone()));
            }
            let bounds = symbols.map(|symbol| VariableBall {
                symbol,
                radius: Real::from(2),
            });
            let context = context_from_problem(&problem);
            let analysis = problem.analyze();
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let report = certify_affine_krawczyk_box(&analysis, &context, &bounds, policy);
                assert_eq!(
                    report.status,
                    AffineKrawczykStatus::CertifiedUniqueRoot,
                    "{order:?} {policy:?}"
                );
                assert_eq!(report.steps.len(), symbols.len());
                let mut root = EvaluationContext::default();
                for ((step, symbol), expected) in
                    report
                        .steps
                        .iter()
                        .zip(symbols)
                        .zip([Real::one(), -c.clone(), c.clone()])
                {
                    assert_eq!(step.symbol, symbol);
                    oracle::assert_equal(&step.certified_root, &expected);
                    root.bind(symbol, step.certified_root.clone());
                }
                for row in &rows {
                    oracle::assert_exact_zero(&row.eval_real(root.bindings()).unwrap());
                }
            }
        }
    }
}

include!("support/hidden_positive.rs");

#[test]
fn unresolved_elimination_factor_survives_row_swaps_and_box_faces() {
    let c = hidden_positive();
    let mut problem = Problem::default();
    let [x, y] = ["x", "y"].map(|name| SymbolId::from(problem.add_variable(name, Real::zero())));
    let row_x = Expr::anonymous_symbol(x) - Expr::int(1);
    let row_y = Expr::real(c.clone()) * Expr::anonymous_symbol(x) + Expr::anonymous_symbol(y);
    // In this order the first row offers an unresolved pivot. The second supplies 1.
    for row in [&row_y, &row_x] {
        problem.add_constraint(Constraint::equality("row", row.clone()));
    }
    let context = context_from_problem(&problem);
    let analysis = problem.analyze();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (radius, expected) in [
            (
                Real::zero(),
                AffineKrawczykStatus::UndecidedContainment { symbol: y },
            ),
            (Real::one(), AffineKrawczykStatus::CertifiedUniqueRoot),
        ] {
            let bounds = [
                VariableBall {
                    symbol: x,
                    radius: Real::one(),
                },
                VariableBall { symbol: y, radius },
            ];
            let report = certify_affine_krawczyk_box(&analysis, &context, &bounds, policy);
            assert_eq!(report.status, expected, "{policy:?}");
            assert_eq!(report.steps[0].symbol, x);
            oracle::assert_equal(&report.steps[0].certified_root, &Real::one());
            if expected == AffineKrawczykStatus::CertifiedUniqueRoot {
                assert_eq!(report.steps.len(), 2);
                assert_eq!(report.steps[1].symbol, y);
                let mut root = EvaluationContext::default();
                for step in report.steps {
                    root.bind(step.symbol, step.certified_root);
                }
                for row in [&row_x, &row_y] {
                    oracle::assert_exact_zero(&row.eval_real(root.bindings()).unwrap());
                }
            } else {
                assert_eq!(report.steps.len(), 1);
            }
        }
    }
}

#[test]
fn point_box_requires_certified_equality_at_the_only_point() {
    let symbol = SymbolId(0);
    let bounds = [VariableBall {
        symbol,
        radius: Real::zero(),
    }];
    for (offset, expected) in [
        (Real::zero(), AffineKrawczykStatus::CertifiedUniqueRoot),
        (
            Real::from(2).powi_i64(-2048).unwrap(),
            AffineKrawczykStatus::RootOutsideBox { symbol },
        ),
        (
            hidden_positive(),
            AffineKrawczykStatus::UndecidedContainment { symbol },
        ),
    ] {
        let mut problem = Problem::default();
        problem.add_variable("x", Real::zero());
        problem.add_constraint(Constraint::equality(
            "x+c",
            Expr::anonymous_symbol(symbol) + Expr::real(offset),
        ));
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = certify_affine_krawczyk_box(&analysis, &context, &bounds, policy);
            assert_eq!(report.status, expected);
            assert_eq!(
                report.steps.len(),
                usize::from(expected == AffineKrawczykStatus::CertifiedUniqueRoot)
            );
        }
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn exact_point_box_keeps_the_certified_comparison_sequence() {
    use hyperreal::dispatch_trace;

    let mut problem = Problem::default();
    let symbol = SymbolId::from(problem.add_variable("x", Real::zero()));
    problem.add_constraint(Constraint::equality("x=0", Expr::anonymous_symbol(symbol)));
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    let bounds = [VariableBall {
        symbol,
        radius: Real::zero(),
    }];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        dispatch_trace::reset();
        let report = dispatch_trace::with_recording(|| {
            certify_affine_krawczyk_box(&analysis, &context, &bounds, policy)
        });
        assert_eq!(report.status, AffineKrawczykStatus::CertifiedUniqueRoot);
        let trace = dispatch_trace::take_trace();
        let counts: Vec<_> = trace
            .dispatch
            .iter()
            .filter(|entry| entry.layer == "hyperlimit" && entry.operation == "compare_reals")
            .map(|entry| (entry.path, entry.count))
            .collect();
        assert_eq!(counts, [("exact-rational", 3)]);
    }
}

#[test]
fn a_point_solving_both_rows_does_not_establish_unique_rank() {
    // The addition identity in hidden_positive makes c exactly 2^-2048.
    // Both rows are multiples of x + 2^2048*y. The origin and (1,-2^-2048)
    // are distinct solutions on the closed unit box.
    let scale = Real::from(2).powi_i64(2048).unwrap();
    let tiny = Real::from(2).powi_i64(-2048).unwrap();
    oracle::assert_equal(&(&scale * &tiny), &Real::one());
    for coefficient in [tiny, hidden_positive()] {
        let mut problem = Problem::default();
        let [x, y] =
            ["x", "y"].map(|name| SymbolId::from(problem.add_variable(name, Real::zero())));
        let rows = [
            Expr::anonymous_symbol(x) + Expr::real(scale.clone()) * Expr::anonymous_symbol(y),
            Expr::real(coefficient) * Expr::anonymous_symbol(x) + Expr::anonymous_symbol(y),
        ];
        for row in &rows {
            problem.add_constraint(Constraint::equality("row", row.clone()));
        }
        let context = context_from_problem(&problem);
        for row in &rows {
            oracle::assert_exact_zero(&row.eval_real(context.bindings()).unwrap());
        }
        let analysis = problem.analyze();
        let bounds = [x, y].map(|symbol| VariableBall {
            symbol,
            radius: Real::one(),
        });
        for policy in [PredicatePolicy::APPROXIMATE_512, PredicatePolicy::STRICT] {
            let report = certify_affine_krawczyk_box(&analysis, &context, &bounds, policy);
            assert_eq!(
                report.status,
                AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot: 1 },
                "{policy:?}"
            );
            assert!(report.steps.is_empty());
            assert_eq!((report.variable_count, report.equality_rows), (2, 2));
        }
    }
}
