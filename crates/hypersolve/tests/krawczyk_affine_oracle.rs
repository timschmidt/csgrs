//! The right-hand side is constructed from a known root and an invertible
//! integer matrix. Closed-box membership can be checked with integers alone.
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    AffineKrawczykStatus, Constraint, Expr, Problem, SymbolId, VariableBall,
    certify_affine_krawczyk_box, context_from_problem,
};

#[test]
fn coupled_affine_certificates_preserve_the_root_and_closed_faces() {
    let x = SymbolId(0);
    let y = SymbolId(1);
    let center = [2_i64, -1];
    for matrix in [[[0, -1], [1, 2]], [[2, 3], [1, -2]], [[1, -2], [-3, 5]]] {
        let [[a, b], [c, d]] = matrix;
        assert_ne!(a * d - b * c, 0);
        for dx in -2..=2 {
            for dy in -2..=2 {
                let root = [center[0] + dx, center[1] + dy];
                let mut p = Problem::default();
                p.add_variable("x", Real::from(center[0]));
                p.add_variable("y", Real::from(center[1]));
                for [a, b] in matrix {
                    let residual = Expr::int(a) * Expr::anonymous_symbol(x)
                        + Expr::int(b) * Expr::anonymous_symbol(y)
                        - Expr::int(a * root[0] + b * root[1]);
                    p.add_constraint(Constraint::equality("row", residual));
                }
                let context = context_from_problem(&p);
                let analysis = p.analyze();
                for (rx, ry) in [(0, 0), (1, 1), (1, 2), (2, 0)] {
                    let bounds = [
                        VariableBall {
                            symbol: x,
                            radius: Real::from(rx),
                        },
                        VariableBall {
                            symbol: y,
                            radius: Real::from(ry),
                        },
                    ];
                    let expected = if dx.abs() > rx {
                        AffineKrawczykStatus::RootOutsideBox { symbol: x }
                    } else if dy.abs() > ry {
                        AffineKrawczykStatus::RootOutsideBox { symbol: y }
                    } else {
                        AffineKrawczykStatus::CertifiedUniqueRoot
                    };
                    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                        let report =
                            certify_affine_krawczyk_box(&analysis, &context, &bounds, policy);
                        assert_eq!(
                            report.status, expected,
                            "matrix {matrix:?}, root {root:?}, radii ({rx},{ry}), {policy:?}"
                        );
                        if expected == AffineKrawczykStatus::CertifiedUniqueRoot {
                            assert_eq!(report.steps.len(), 2);
                            for (step, coordinate) in report.steps.iter().zip(root) {
                                assert_eq!(step.certified_root, Real::from(coordinate));
                            }
                        }
                    }
                }
            }
        }
    }
}
