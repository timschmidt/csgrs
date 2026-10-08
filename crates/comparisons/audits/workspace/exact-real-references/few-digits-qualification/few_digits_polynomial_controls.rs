// Compile the unchanged, self-contained Hypersolve symbolic module against
// the frozen Hyperreal snapshot. This is a scoped source-module check, not
// a claim that the entire current Hypersolve integration suite ran.
#[allow(dead_code)]
#[path = "/home/tim/Documents/GitHub/workspace/hypersolve/src/symbolic.rs"]
mod symbolic;
use hyperreal::{Rational, Real};
use symbolic::{Expr, SymbolId};
use std::collections::HashMap;

fn main() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let f = x.clone() * x.clone() * y.clone();
    let dx = f.derivative(SymbolId(0));
    let dy = f.derivative(SymbolId(1));
    let dxy = dx.derivative(SymbolId(1));
    let dyy = dy.derivative(SymbolId(1));
    let zero = Expr::zero().derivative(SymbolId(0));
    let canceled = (f.clone() - f).derivative(SymbolId(1));
    let mut checks = 0;
    for a in -4..=4 {
        for b in -4..=4 {
            let bindings = HashMap::from([
                (SymbolId(0), Real::new(Rational::new(a))),
                (SymbolId(1), Real::new(Rational::new(b))),
            ]);
            for (expr, q) in [(&dx, 2*a*b), (&dy, a*a), (&dxy, 2*a),
                (&dyy, 0), (&zero, 0), (&canceled, 0)] {
                assert_eq!(expr.eval_real(&bindings).unwrap(), Real::new(Rational::new(q)));
                checks += 1;
            }
        }
    }
    println!("PASS {checks} exact Hypersolve symbolic-module derivative/zero controls");
}
