//! Enclose an oscillator's expressions and symbolic Jacobian over a state box.
//! This is a field-image calculation, not an existence/uniqueness certificate.
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn d(a: i64, b: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), exponent).unwrap()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let [time, position, velocity] = [SymbolId(0), SymbolId(1), SymbolId(2)];
    let field = [
        Expr::anonymous_symbol(velocity),
        -Expr::anonymous_symbol(position),
    ];
    let mut outputs = field.to_vec();
    for component in &field {
        for variable in [position, velocity] {
            // Symbolic derivative construction still has a separate resource
            // gap. It is not included in the graph/evaluation budgets below.
            outputs.push(component.derivative(variable));
        }
    }
    let mut graph = ExpressionBudget::new(100, 1_000);
    let program = ExpressionProgram::compile(&outputs, &[time, position, velocity], &mut graph)?;
    let state = [d(0, 1, -2), d(1, 2, 0), d(-1, 1, 0)];
    let result = program.eval_dyadic(
        &state,
        -32,
        &mut graph,
        &mut DyadicBudget::new(16_384, 1_000_000_000),
    )?;
    assert_eq!(
        result,
        [
            d(-1, 1, 0),
            d(-2, -1, 0),
            d(0, 0, 0),
            d(1, 1, 0),
            d(-1, -1, 0),
            d(0, 0, 0)
        ]
    );
    println!("field: ([-1,1], [-2,-1]); Jacobian: [[0,1], [-1,0]]");
    Ok(())
}
