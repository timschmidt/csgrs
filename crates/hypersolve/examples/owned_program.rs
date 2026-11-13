//! Own and replay an oscillator's source and symbolic Jacobian definitions.
//! This evaluates exact points; it does not certify a trajectory or interval.

use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let time = SymbolId(0);
    let position = SymbolId(1);
    let velocity = SymbolId(2);
    let field = [
        Expr::anonymous_symbol(velocity),
        -Expr::anonymous_symbol(position),
    ];
    let mut outputs = field.to_vec();
    for component in &field {
        for variable in [position, velocity] {
            // Existing symbolic authority; derivative construction is outside
            // the program's structural graph budget.
            outputs.push(component.derivative(variable));
        }
    }
    let mut budget = ExpressionBudget::new(100, 1_000);
    let program = ExpressionProgram::compile(&outputs, &[time, position, velocity], &mut budget)?;
    drop(outputs);
    let replay = ExpressionProgram::from_definition(program.definition().clone(), &mut budget)?;
    assert_eq!(program.definition(), replay.definition());
    let values = replay.eval_real(&[Real::zero(), Real::from(2), Real::from(3)], &mut budget)?;
    assert_eq!(values, [3, -2, 0, 1, -1, 0].map(Real::from));
    println!("exact field: (3, -2); Jacobian: [[0, 1], [-1, 0]]");
    Ok(())
}
