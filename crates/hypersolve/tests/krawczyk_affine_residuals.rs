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
