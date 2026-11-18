//! For k((x-t)²-n), the roots are t ± sqrt(n). At either root, a ball of
//! radius m*sqrt(n) has contraction factor m and image radius m²*sqrt(n).
#[path = "support/represented_roots.rs"]
mod oracle;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::Real;
use hypersolve::{
    Constraint, EvaluationContext, Expr, Problem, QuadraticKrawczykStatus as Status, SymbolId,
    VariableBall, certify_univariate_quadratic_krawczyk_box, context_from_problem,
};
use std::cmp::Ordering;

#[test]
fn algebraic_center_and_radius_preserve_the_signed_derivative_and_original_equation() {
    for (n, translation, scale) in [(2, 0, 1_i64), (3, -5, -2), (5, 4, 3)] {
        let root = oracle::sqrt(n);
        for sign in [-1_i64, 1] {
            let center = Real::from(translation) + Real::from(sign) * &root;
            let mut problem = Problem::default();
            let symbol = SymbolId::from(problem.add_variable("x", center.clone()));
            let shifted = Expr::anonymous_symbol(symbol) - Expr::int(translation);
            let original = Expr::int(scale) * (shifted.clone() * shifted - Expr::int(n));
            problem.add_constraint(Constraint::equality(
                "translated quadratic",
                original.clone(),
            ));
            let analysis = problem.analyze();
            let context = context_from_problem(&problem);
            // The image is contained up to and including m = 1. Contraction is strict.
            for quarters in [0_i64, 1, 3, 4, 5, 8] {
                let radius = &root * oracle::q(quarters, 4);
                let bounds = [VariableBall {
                    symbol,
                    radius: radius.clone(),
                }];
                let expected = match quarters.cmp(&4) {
                    Ordering::Less => Status::CertifiedUniqueRoot,
                    Ordering::Equal => Status::NonContractiveDerivative,
                    Ordering::Greater => Status::ImageOutsideBox,
                };
                for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                    let report = certify_univariate_quadratic_krawczyk_box(
                        &analysis, &context, &bounds, policy,
                    );
                    assert_eq!(report.rows.len(), 1);
                    assert_eq!(report.examined_rows, 1);
                    assert_eq!(report.certified_rows, usize::from(quarters < 4));
                    let row = &report.rows[0];
                    assert_eq!(row.status, expected, "{n} {scale} {sign} {quarters}");
                    assert_eq!((row.constraint_index, row.symbol), (0, symbol));
                    let step = row.step.as_ref().unwrap();
                    let mut replay = EvaluationContext::default();
                    replay.bind(symbol, &row.candidate + step);
                    let replayed = original.eval_real(replay.bindings()).unwrap();
                    // The independent values use the factored source and the known
                    // root. A translated radical can need a proof of equality even
                    // when the computed residual has no structural zero flag.
                    for (field, actual, expected) in [
                        ("center", &row.candidate, center.clone()),
                        ("radius", &row.radius, radius.clone()),
                        ("residual", &row.residual, Real::zero()),
                        ("step", step, Real::zero()),
                        (
                            "derivative",
                            &row.derivative,
                            Real::from(2 * scale * sign) * &root,
                        ),
                        (
                            "remainder",
                            row.remainder_radius.as_ref().unwrap(),
                            &root * oracle::q(quarters * quarters, 16),
                        ),
                        (
                            "numerator",
                            row.contraction_numerator.as_ref().unwrap(),
                            Real::from(2 * scale.abs()) * &radius,
                        ),
                        (
                            "denominator",
                            row.contraction_denominator.as_ref().unwrap(),
                            Real::from(2 * scale.abs()) * &root,
                        ),
                        ("replayed residual", &replayed, Real::zero()),
                    ] {
                        assert!(
                            matches!(
                                compare_reals(actual, &expected, PredicatePolicy::STRICT),
                                PredicateOutcome::Decided {
                                    value: Ordering::Equal,
                                    certainty: Certainty::Exact | Certainty::Filtered,
                                    ..
                                }
                            ),
                            "{field}, n={n}, k={scale}, sign={sign}, quarters={quarters}"
                        );
                    }
                }
            }
        }
    }
}
