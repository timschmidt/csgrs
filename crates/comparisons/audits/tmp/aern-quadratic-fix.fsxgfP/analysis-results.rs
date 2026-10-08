use hyperreal::Real;
use hypersolve::{Constraint, Expr, Problem, SymbolId};

fn main() {
    for multivariate in [false, true] {
        let mut problem = Problem::default();
        for i in 0..8 {
            problem.add_variable(format!("x{i}"), Real::from(i + 1));
        }
        let x = Expr::symbol(SymbolId(0), "x0");
        let y = Expr::symbol(SymbolId(1), "x1");
        for i in 1..=16 {
            let residual = if multivariate {
                x.clone() * y.clone() * Expr::int(i)
                    + x.clone().powi(2) * Expr::int(i + 1)
                    - y.clone() * Expr::int(i + 2) + Expr::int(i)
            } else {
                x.clone() * x.clone() * Expr::int(i)
                    - x.clone() * Expr::int(2 * i) + Expr::int(i)
            };
            problem.add_constraint(Constraint::equality(format!("row{i}"), residual));
        }
        println!("{:#?}", problem.analyze());
    }
}
