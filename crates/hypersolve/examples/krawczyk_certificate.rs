//! Certify a root box for x² = 2 and x² + y² = 4 using exact rational input.

use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, Expr, MultivariateQuadraticKrawczykStatus, Problem, VariableBall,
    certify_multivariate_quadratic_krawczyk_box, context_from_problem,
};

fn ratio(n: i64, d: u64) -> Real {
    Real::from(Rational::fraction(n, d).unwrap())
}

fn main() {
    let mut problem = Problem::default();
    let x = problem.add_variable("x", ratio(3, 2)).into();
    let y = problem.add_variable("y", ratio(3, 2)).into();
    let x_expr = Expr::symbol(x, "x");
    let y_expr = Expr::symbol(y, "y");
    problem.add_constraint(Constraint::equality(
        "x² = 2",
        x_expr.clone().powi(2) - Expr::real(2.into()),
    ));
    problem.add_constraint(Constraint::equality(
        "x² + y² = 4",
        x_expr.powi(2) + y_expr.powi(2) - Expr::real(4.into()),
    ));

    let analysis = problem.analyze();
    let center = context_from_problem(&problem);
    let certify = |y_radius| {
        let bounds = [
            VariableBall {
                symbol: x,
                radius: ratio(1, 4),
            },
            VariableBall {
                symbol: y,
                radius: y_radius,
            },
        ];
        certify_multivariate_quadratic_krawczyk_box(
            &analysis,
            &center,
            &bounds,
            PredicatePolicy::STRICT,
        )
    };

    let report = certify(ratio(1, 4));
    assert_eq!(
        report.status,
        MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot
    );
    assert_eq!(report.variables.len(), 2);
    // Both Newton displacements are -1/12. The two residual remainders
    // are bounded by 1/16 and 1/8. Applying the absolute inverse Jacobian
    // gives different image radii for x and y.
    for (coordinate, (name, symbol, image, contraction)) in report.variables.iter().zip([
        ("x", x, ratio(5, 48), ratio(1, 6)),
        ("y", y, ratio(7, 48), ratio(1, 2)),
    ]) {
        assert_eq!(coordinate.symbol, symbol);
        assert_eq!(coordinate.step, ratio(-1, 12));
        assert_eq!(coordinate.image_radius, image);
        assert_eq!(coordinate.contraction_bound, contraction);
        println!(
            "{name} step {}, image radius {}, contraction {}",
            coordinate.step, coordinate.image_radius, coordinate.contraction_bound
        );
    }
    println!("Certified one root in [5/4, 7/4] × [5/4, 7/4].");

    // The tighter y interval [45/32, 51/32] still contains sqrt(2), since
    // 45² < 2·32² < 51². The computed y image still exceeds its radius.
    let tighter = certify(ratio(3, 32));
    assert_eq!(
        tighter.status,
        MultivariateQuadraticKrawczykStatus::ImageOutsideBox { symbol: y }
    );
    println!("With y radius 3/32 the image extends outside the box on y.");
}
