use criterion::Criterion;
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, Expr, Problem, QuadraticResidual, SymbolId, UnivariateQuadraticResidual,
};
use std::collections::HashMap;
use std::hint::black_box;

fn symbol(index: u32) -> Expr {
    Expr::symbol(SymbolId(index), format!("x{index}"))
}

pub fn quadratic_extraction(c: &mut Criterion) {
    let mut group = c.benchmark_group("quadratic_extraction");
    for case in [
        "uni_square",
        "uni_factored",
        "uni_wide",
        "uni_pi",
        "uni_cancellation",
        "uni_scaled_cancellation",
        "uni_bad_degree",
        "multi_square",
        "multi_cross",
        "multi_dense8",
        "multi_dense32",
        "multi_zero_terms",
        "multi_pi",
        "multi_bad_degree",
        "multi_bad_monomial",
        "multi_cancellation",
    ] {
        let count = if case == "multi_dense32" { 32 } else { 8 };
        let mut problem = Problem::default();
        let mut bindings = HashMap::new();
        for index in 0..count {
            problem.add_variable(format!("x{index}"), Real::from(index + 1));
            bindings.insert(SymbolId(index), Real::from(index + 1));
        }
        let x = symbol(0);
        let y = symbol(1);
        let cubic = x.clone().powi(2) * x.clone();
        let expression = match case {
            "uni_square" | "multi_square" => x.clone().powi(2) - Expr::int(2),
            "uni_factored" => (x.clone() + Expr::int(3)) * (x.clone() - Expr::int(2)),
            "uni_wide" => {
                let wide = Real::from(Rational::fraction(i64::MAX, 1_000_000_007).unwrap());
                (x.clone() - Expr::real(wide)).powi(2)
            }
            "uni_pi" | "multi_pi" => (x.clone() - Expr::real(Real::pi())).powi(2),
            "uni_cancellation" | "multi_cancellation" => {
                cubic.clone() - x.clone() * x.clone().powi(2) + x.clone()
            }
            "uni_scaled_cancellation" => {
                cubic.clone() * Expr::int(2) - cubic.clone() - cubic + x.clone()
            }
            "uni_bad_degree" | "multi_bad_degree" => {
                x.clone().powi(2) * (x.clone() - x.clone().powi(2))
            }
            "multi_bad_monomial" => x.clone().powi(2) * (x.clone() - y.clone()),
            "multi_cross" => {
                x.clone() * y.clone() * Expr::int(5) + x.clone().powi(2) * Expr::int(2)
                    - y.clone() * Expr::int(7)
                    + Expr::int(11)
            }
            "multi_dense8" | "multi_dense32" => {
                (1..count).fold(x.clone(), |sum, i| sum + symbol(i)).powi(2)
            }
            "multi_zero_terms" => (x.clone() - x + y.clone() - y + Expr::int(1)).powi(2),
            _ => unreachable!(),
        };
        let expected_some = !case.contains("bad_");
        // Verify successful work and deliberate rejection outside timing.
        if case.starts_with("uni_") {
            let extracted = UnivariateQuadraticResidual::from_expr(&expression, &problem);
            assert_eq!(extracted.is_some(), expected_some, "{case}");
            if let Some(extracted) = extracted {
                if case == "uni_pi" {
                    // Comparing expanded and unexpanded computable expressions
                    // with PartialEq need not decide their mathematical equality.
                    assert_eq!(extracted.constant(), &(&Real::pi() * &Real::pi()));
                    assert_eq!(extracted.linear(), &(Real::from(-2) * Real::pi()));
                    assert_eq!(extracted.quadratic(), &Real::one());
                } else {
                    assert_eq!(
                        extracted.eval_real(&problem.variables, &bindings).unwrap(),
                        expression.eval_real(&bindings).unwrap(),
                        "{case}"
                    );
                }
            }
            group.bench_function(case, |b| {
                b.iter(|| {
                    UnivariateQuadraticResidual::from_expr(
                        black_box(&expression),
                        black_box(&problem),
                    )
                });
            });
        } else {
            let extracted = QuadraticResidual::from_expr(&expression, &problem);
            assert_eq!(extracted.is_some(), expected_some, "{case}");
            if let Some(extracted) = extracted {
                if case == "multi_pi" {
                    assert_eq!(extracted.constant(), &(&Real::pi() * &Real::pi()));
                    assert_eq!(extracted.linear_terms().len(), 1);
                    assert_eq!(extracted.linear_terms()[0].symbol, SymbolId(0));
                    assert_eq!(
                        extracted.linear_terms()[0].coefficient,
                        Real::from(-2) * Real::pi()
                    );
                    assert_eq!(extracted.quadratic_terms().len(), 1);
                    assert_eq!(extracted.quadratic_terms()[0].first, SymbolId(0));
                    assert_eq!(extracted.quadratic_terms()[0].second, SymbolId(0));
                    assert_eq!(extracted.quadratic_terms()[0].coefficient, Real::one());
                } else {
                    assert_eq!(
                        extracted.eval_real(&problem.variables, &bindings).unwrap(),
                        expression.eval_real(&bindings).unwrap(),
                        "{case}"
                    );
                }
            }
            group.bench_function(case, |b| {
                b.iter(|| {
                    QuadraticResidual::from_expr(black_box(&expression), black_box(&problem))
                });
            });
        }
    }
    for (case, multivariate) in [("analyze_uni16", false), ("analyze_multi16", true)] {
        let mut problem = Problem::default();
        for index in 0..8 {
            problem.add_variable(format!("x{index}"), Real::from(index + 1));
        }
        let x = symbol(0);
        let y = symbol(1);
        for index in 1..=16 {
            let residual = if multivariate {
                x.clone() * y.clone() * Expr::int(index) + x.clone().powi(2) * Expr::int(index + 1)
                    - y.clone() * Expr::int(index + 2)
                    + Expr::int(index)
            } else {
                x.clone() * x.clone() * Expr::int(index) - x.clone() * Expr::int(2 * index)
                    + Expr::int(index)
            };
            problem.add_constraint(Constraint::equality(format!("row{index}"), residual));
        }
        let analysis = problem.analyze();
        assert!(analysis.quadratic_residuals().iter().all(Option::is_some));
        assert_eq!(
            analysis
                .univariate_quadratic_residuals()
                .iter()
                .all(Option::is_some),
            !multivariate
        );
        group.bench_function(case, |b| b.iter(|| black_box(&problem).analyze()));
    }
    group.finish();
}
