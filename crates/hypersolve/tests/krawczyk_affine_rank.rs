//! A singular affine system may be inconsistent or have infinitely many roots.
//! The first missing pivot is known directly from its two integer columns.

use hyperreal::Real;
use hypersolve::{
    AffineKrawczykStatus, Constraint, Expr, PredicatePolicy, Problem, SymbolId, VariableBall,
    certify_affine_krawczyk_box, context_from_problem,
};

#[test]
fn deficient_integer_matrices_report_the_first_missing_pivot() {
    let rows = (-2..=2).flat_map(|u| (-2..=2).map(move |v| [u, v]));
    let mut singular = 0;
    for [a, b] in rows.clone() {
        for [c, d] in rows.clone() {
            if a * d == b * c {
                check_singular_matrix([[a, b], [c, d]]);
                singular += 1;
            }
        }
    }
    // Count the zero matrix, 24 nonzero first rows paired with a zero
    // second row, 24 with a zero first row, and 80 proportional pairs.
    assert_eq!(singular, 129);
}

fn check_singular_matrix(matrix: [[i64; 2]; 2]) {
    let [[a, b], [c, d]] = matrix;
    let (pivot, column) = if [a, c] == [0, 0] {
        (0, [b, d])
    } else {
        (1, [a, c])
    };
    // A*(1,1) lies in the image. A nonzero perpendicular to its spanning
    // column is outside the image. The zero matrix only maps to the origin.
    let incompatible = if column == [0, 0] {
        [1, 0]
    } else {
        [-column[1], column[0]]
    };
    for rhs in [[a + b, c + d], incompatible] {
        let mut problem = Problem::default();
        let [x, y] =
            ["x", "y"].map(|name| SymbolId::from(problem.add_variable(name, Real::zero())));
        for ([u, v], value) in matrix.into_iter().zip(rhs) {
            let residual = Expr::int(u) * Expr::anonymous_symbol(x)
                + Expr::int(v) * Expr::anonymous_symbol(y)
                - Expr::int(value);
            problem.add_constraint(Constraint::equality("row", residual));
        }
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        let balls = [x, y].map(|symbol| VariableBall {
            symbol,
            radius: Real::one(),
        });
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = certify_affine_krawczyk_box(&analysis, &context, &balls, policy);
            assert_eq!(
                report.status,
                AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot },
                "matrix={matrix:?}, rhs={rhs:?}, policy={policy:?}"
            );
            assert!(report.steps.is_empty());
            assert_eq!((report.variable_count, report.equality_rows), (2, 2));
        }
    }
}
