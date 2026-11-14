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

#[test]
fn high_degree_algebraic_coefficients_retain_independent_values_and_derivatives() {
    // P=sqrt(2)*u^34*v^3+3, W=1+v. This is outside a low-degree catalog.
    let alpha = Real::from(2).sqrt().unwrap();
    let mut coefficients = vec![vec![]; 35];
    coefficients[0] = vec![3.into()];
    coefficients[34] = vec![0.into(), 0.into(), 0.into(), alpha.clone()];
    let program = ExpressionProgram::from_bivariate_quotients(
        &[BivariatePolynomial::new(coefficients)],
        &BivariatePolynomial::new(vec![vec![1.into(), 1.into()]]),
        [SymbolId(8), SymbolId(3)],
        &mut budget(),
    )
    .unwrap();
    let jacobian = program
        .jacobian(&[SymbolId(8), SymbolId(3)], &mut budget())
        .unwrap();
    let values = jacobian
        .program()
        .eval_real(&[1.into(), 1.into()], &mut budget())
        .unwrap();
    let expected = [
        (&alpha + Real::from(3)) / Real::from(2),
        Ok(Real::from(17) * &alpha),
        (Real::from(5) * &alpha - Real::from(3)) / Real::from(4),
    ];
    for (slot, expected) in [
        0,
        jacobian.partial_output(0, 0).unwrap(),
        jacobian.partial_output(0, 1).unwrap(),
    ]
    .into_iter()
    .zip(expected)
    {
        assert_eq!(
            hyperlimit::compare_reals(
                &values[slot],
                &expected.unwrap(),
                hyperlimit::PredicatePolicy::STRICT
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
}

#[test]
fn empty_output_maps_still_validate_ordered_bindings() {
    let program = ExpressionProgram::from_bivariate_quotients(
        &[],
        &BivariatePolynomial::new(vec![]),
        [SymbolId(7), SymbolId(2)],
        &mut budget(),
    )
    .unwrap();
    assert_eq!(program.definition().variables, [SymbolId(7), SymbolId(2)]);
    assert!(
        program
            .eval_real(&[1.into(), 2.into()], &mut budget())
            .unwrap()
            .is_empty()
    );
    for arguments in [vec![], vec![1.into()], vec![1.into(), 2.into(), 3.into()]] {
        assert!(
            matches!(program.eval_real(&arguments,&mut budget()),Err(ExpressionError::BindingCount{expected:2,actual}) if actual==arguments.len())
        );
    }
}

#[cfg(feature = "serde")]
#[test]
fn native_definition_replay_preserves_shared_denominator_and_rejects_bad_references() {
    let source = ExpressionProgram::from_bivariate_quotients(
        &[
            BivariatePolynomial::new(vec![vec![], vec![2.into()]]),
            BivariatePolynomial::new(vec![]),
        ],
        &BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]),
        [SymbolId(4), SymbolId(9)],
        &mut budget(),
    )
    .unwrap();
    let bytes = serde_json::to_string(source.definition()).unwrap();
    let definition = serde_json::from_str(&bytes).unwrap();
    let replay = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
    assert_eq!(source.definition(), replay.definition());
    assert_eq!(serde_json::to_string(replay.definition()).unwrap(), bytes);
    assert_eq!(
        replay
            .eval_real(&[3.into(), 2.into()], &mut budget())
            .unwrap(),
        [Real::from(3), Real::from(0)]
    );
    assert!(matches!(
        replay.eval_real(&[3.into(), 0.into()], &mut budget()),
        Err(ExpressionError::Scalar { .. })
    ));
    let mut invalid = source.definition().clone();
    let output = invalid.outputs[0];
    let ExpressionNode::Binary { right, .. } = &mut invalid.nodes[output] else {
        panic!("division guard")
    };
    *right = output;
    assert!(
        matches!(ExpressionProgram::from_definition(invalid,&mut budget()),Err(ExpressionError::InvalidReference{node,input}) if node==output && input==output)
    );
}
