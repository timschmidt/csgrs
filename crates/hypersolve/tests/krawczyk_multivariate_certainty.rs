//! A Krawczyk coordinate uses both a closed containment test and a strict
//! contraction test. Terminal rounding at either boundary can change its meaning.
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, MultivariateQuadraticKrawczykReport,
    MultivariateQuadraticKrawczykStatus as Status, Problem, SymbolId, VariableBall,
    certify_multivariate_quadratic_krawczyk_box, context_from_problem,
};
use std::cmp::Ordering;

include!("support/hidden_positive.rs");

fn certify(c: Real, radius: Real, policy: PredicatePolicy) -> MultivariateQuadraticKrawczykReport {
    let mut problem = Problem::default();
    let symbol = SymbolId(0);
    problem.add_variable("x", Real::zero());
    let x = Expr::anonymous_symbol(symbol);
    problem.add_constraint(Constraint::equality(
        "x²+x+c",
        x.clone() * x.clone() + (x + Expr::real(c)),
    ));
    certify_multivariate_quadratic_krawczyk_box(
        &problem.analyze(),
        &context_from_problem(&problem),
        &[VariableBall { symbol, radius }],
        policy,
    )
}

#[test]
fn point_box_requires_a_proof_of_containment() {
    let c = hidden_positive();
    assert!(matches!(
        compare_reals(&c, &Real::zero(), PredicatePolicy::APPROXIMATE_512),
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: Certainty::Approximate,
            ..
        }
    ));
    // At the only point in the box, x = 0, the residual equals c > 0.
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let report = certify(c.clone(), Real::zero(), policy);
        assert_eq!(
            report.status,
            Status::Undecided {
                symbol: SymbolId(0)
            }
        );
        assert!(!report.certified_unique_root());
    }
}

#[test]
fn strict_contraction_uses_the_exact_side_of_one() {
    let half = Real::from(2).powi_i64(-1).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        // With f(x)=x²+x, image radius = r² and derivative variation = 2r.
        for (radius, expected) in [
            (Real::zero(), Status::CertifiedUniqueRoot),
            (
                Real::from(2).powi_i64(-2).unwrap(),
                Status::CertifiedUniqueRoot,
            ),
            (
                half.clone(),
                Status::NonContractiveDerivative {
                    symbol: SymbolId(0),
                },
            ),
            (
                half.clone() - hidden_positive(),
                Status::Undecided {
                    symbol: SymbolId(0),
                },
            ),
        ] {
            assert_eq!(certify(Real::zero(), radius, policy).status, expected);
        }
    }
}

#[test]
fn coupled_quadratic_rows_keep_the_inverse_relation() {
    let mut problem = Problem::default();
    let sx = SymbolId(0);
    let sy = SymbolId(1);
    problem.add_variable("x", Real::zero());
    problem.add_variable("y", Real::zero());
    let x = Expr::anonymous_symbol(sx);
    let y = Expr::anonymous_symbol(sy);
    let a = Expr::real(Real::from(2).powi_i64(-4).unwrap());
    problem.add_constraint(Constraint::equality(
        "x+x²/16-1",
        x.clone() + a * x.clone() * x.clone() - Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "c*x+y+y²",
        Expr::real(hidden_positive()) * x + y.clone() + y.clone() * y,
    ));
    // y = 0 forces x = 0 in the second equation, which violates the first.
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    let bounds = [
        VariableBall {
            symbol: sx,
            radius: Real::from(2),
        },
        VariableBall {
            symbol: sy,
            radius: Real::zero(),
        },
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert_eq!(
            certify_multivariate_quadratic_krawczyk_box(&analysis, &context, &bounds, policy)
                .status,
            Status::Undecided { symbol: sy }
        );
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn exact_point_box_keeps_its_four_certified_comparisons() {
    use hyperreal::dispatch_trace;
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        dispatch_trace::reset();
        let report = dispatch_trace::with_recording(|| certify(Real::zero(), Real::zero(), policy));
        assert_eq!(report.status, Status::CertifiedUniqueRoot);
        let snapshot = dispatch_trace::take_trace();
        let counts: Vec<_> = snapshot
            .dispatch
            .iter()
            .filter(|e| e.layer == "hyperlimit" && e.operation == "compare_reals")
            .map(|e| (e.path, e.count))
            .collect();
        assert_eq!(counts, [("exact-rational", 4)]);
    }
}
