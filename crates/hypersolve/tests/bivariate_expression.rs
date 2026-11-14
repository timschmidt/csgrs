#![cfg(feature = "expression-program")]

use hyperreal::Real;
use hypersolve::{
    BivariatePolynomial, SymbolId,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionError, ExpressionNode, ExpressionProgram,
    },
    work_budget::CancellationToken,
};

fn budget() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 1_000_000)
}

#[test]
fn empty_ragged_polynomials_keep_shared_original_denominator_guards() {
    let zero = BivariatePolynomial::new(Vec::new());
    let ragged = BivariatePolynomial::new(vec![vec![], vec![], vec![5.into()]]);
    // W=v, so the second output is 5u²/v and the first remains 0/v.
    let w = BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]);
    let program = ExpressionProgram::from_bivariate_quotients(
        &[zero, ragged],
        &w,
        [SymbolId(0), SymbolId(1)],
        &mut budget(),
    )
    .unwrap();
    assert_eq!(
        program
            .eval_real(&[2.into(), 4.into()], &mut budget())
            .unwrap(),
        vec![Real::from(0), Real::from(5)]
    );
    let divisors: Vec<_> = program
        .definition()
        .outputs
        .iter()
        .map(|&output| {
            let ExpressionNode::Binary {
                operation: BinaryOperation::Div,
                right,
                ..
            } = program.definition().nodes[output]
            else {
                panic!("each output keeps its division guard")
            };
            right
        })
        .collect();
    assert_eq!(divisors[0], divisors[1]);
    let error = program
        .eval_real(&[2.into(), 0.into()], &mut budget())
        .unwrap_err();
    assert!(matches!(error, ExpressionError::Scalar { .. }));
    assert!(std::error::Error::source(&error).is_some());
    let replay =
        ExpressionProgram::from_definition(program.definition().clone(), &mut budget()).unwrap();
    assert_eq!(replay.definition(), program.definition());
    assert!(matches!(
        replay.eval_real(&[2.into(), 0.into()], &mut budget()),
        Err(ExpressionError::Scalar { .. })
    ));
}

#[test]
fn construction_budget_boundaries_cancellation_and_retry_are_explicit() {
    let p = BivariatePolynomial::new(vec![vec![1.into(), 2.into()], vec![3.into()]]);
    let w = BivariatePolynomial::new(vec![vec![1.into()]]);
    let build = |budget: &mut ExpressionBudget| {
        ExpressionProgram::from_bivariate_quotients(
            std::slice::from_ref(&p),
            &w,
            [SymbolId(0), SymbolId(1)],
            budget,
        )
    };
    let mut ample = budget();
    let program = build(&mut ample).unwrap();
    let work = 1_000_000 - ample.remaining_work();
    assert!(work > 0);
    for limit in 0..work {
        assert!(
            matches!(
                build(&mut ExpressionBudget::new(100_000, limit)),
                Err(ExpressionError::WorkExhausted)
            ),
            "work limit {limit}"
        );
    }
    assert_eq!(
        build(&mut ExpressionBudget::new(100_000, work))
            .unwrap()
            .definition(),
        program.definition()
    );
    let definition = program.definition();
    let entries = definition.variables.len() + definition.nodes.len() + definition.outputs.len();
    for limit in 0..entries {
        assert!(
            matches!(
                build(&mut ExpressionBudget::new(limit, 1_000_000)),
                Err(ExpressionError::SizeExhausted)
            ),
            "entry limit {limit}"
        );
    }
    assert_eq!(
        build(&mut ExpressionBudget::new(entries, work))
            .unwrap()
            .definition(),
        definition
    );
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        build(&mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(cancelled.remaining_work(), 1_000_000);
    assert_eq!(build(&mut budget()).unwrap().definition(), definition);
    assert!(matches!(
        ExpressionProgram::from_bivariate_quotients(&[], &w, [SymbolId(9); 2], &mut budget()),
        Err(ExpressionError::DuplicateVariable(SymbolId(9)))
    ));
}

#[test]
fn bivariate_quotients_preserve_axis_order_and_independent_derivatives() {
    // P(u,v) = 1 + 2v + 3u + 4u²v; W(u,v) = 1 + u.
    // The ragged middle row and asymmetric coefficients expose transposition.
    let p = BivariatePolynomial::new(vec![
        vec![1.into(), 2.into()],
        vec![3.into()],
        vec![0.into(), 4.into()],
    ]);
    let w = BivariatePolynomial::new(vec![vec![1.into()], vec![1.into()]]);
    let program = ExpressionProgram::from_bivariate_quotients(
        &[p],
        &w,
        [SymbolId(7), SymbolId(2)],
        &mut budget(),
    )
    .unwrap();
    // P(1,2)=16, W=2. P_u=19, P_v=6, so Q_u=11/2, Q_v=3.
    assert_eq!(
        program
            .eval_real(&[1.into(), 2.into()], &mut budget())
            .unwrap(),
        vec![Real::from(8)]
    );
    let jacobian = program
        .jacobian(&[SymbolId(7), SymbolId(2)], &mut budget())
        .unwrap();
    let values = jacobian
        .program()
        .eval_real(&[1.into(), 2.into()], &mut budget())
        .unwrap();
    assert_eq!(values[0], Real::from(8));
    assert_eq!(
        values[jacobian.partial_output(0, 0).unwrap()],
        (Real::from(11) / Real::from(2)).unwrap()
    );
    assert_eq!(
        values[jacobian.partial_output(0, 1).unwrap()],
        Real::from(3)
    );
}
