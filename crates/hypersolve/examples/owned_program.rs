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
    let mut budget = ExpressionBudget::new(100, 1_000);
    let source = ExpressionProgram::compile(&field, &[time, position, velocity], &mut budget)?;
    let jacobian = source.jacobian(&[position, velocity], &mut budget)?;
    let replay_source =
        ExpressionProgram::from_definition(jacobian.source().definition().clone(), &mut budget)?;
    let replay = replay_source.jacobian(jacobian.variables(), &mut budget)?;
    assert_eq!(
        jacobian.program().definition(),
        replay.program().definition()
    );
    let values = replay
        .program()
        .eval_real(&[Real::zero(), Real::from(2), Real::from(3)], &mut budget)?;
    assert_eq!(values[..2], [3, -2].map(Real::from));
    for (row, entries) in [[0, 1], [-1, 0]].into_iter().enumerate() {
        for (column, entry) in entries.into_iter().enumerate() {
            assert_eq!(
                values[replay.partial_output(row, column).unwrap()],
                Real::from(entry)
            );
        }
    }
    println!("exact field: (3, -2); Jacobian: [[0, 1], [-1, 0]]");
    Ok(())
}
