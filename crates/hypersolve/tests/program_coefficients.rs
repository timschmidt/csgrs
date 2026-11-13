#![cfg(feature = "taylor-model")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoefficientRequest, ExpressionBudget, ExpressionProgram},
};
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
#[test]
fn requested_mixed_coefficients_are_normalized_and_bind_original_source() {
    let (x, y) = (SymbolId(3), SymbolId(9));
    let source = ExpressionProgram::compile(
        &[Expr::anonymous_symbol(x).powi(2) * Expr::anonymous_symbol(y)],
        &[y, x],
        &mut budget(),
    )
    .unwrap();
    let requests = [
        CoefficientRequest {
            output: 0,
            powers: vec![(x, 1)],
        },
        CoefficientRequest {
            output: 0,
            powers: vec![(x, 2)],
        },
        CoefficientRequest {
            output: 0,
            powers: vec![(x, 1), (y, 1)],
        },
        CoefficientRequest {
            output: 0,
            powers: vec![(x, 2), (y, 1)],
        },
        CoefficientRequest {
            output: 0,
            powers: vec![(y, 2)],
        },
    ];
    let coefficients = source.coefficients(&requests, &mut budget()).unwrap();
    assert_eq!(coefficients.source().definition(), source.definition());
    let values = coefficients
        .program()
        .eval_real(&[Real::from(3), Real::from(2)], &mut budget())
        .unwrap();
    assert_eq!(values, [12, 12, 3, 4, 1, 0].map(Real::from));
}

#[test]
fn polynomial_coefficients_match_independent_binomial_controls() {
    fn choose(n: usize, k: usize) -> i64 {
        if k > n {
            return 0;
        }
        (0..k).fold(1_i64, |v, i| v * (n - i) as i64 / (i + 1) as i64)
    }
    let (x, y) = (SymbolId(3), SymbolId(9));
    for m in 0..=4 {
        for n in 0..=4 {
            let source = ExpressionProgram::compile(
                &[(if m == 0 {
                    Expr::int(1)
                } else {
                    Expr::anonymous_symbol(x).powi(m as i64)
                }) * (if n == 0 {
                    Expr::int(1)
                } else {
                    Expr::anonymous_symbol(y).powi(n as i64)
                })],
                &[y, x],
                &mut budget(),
            )
            .unwrap();
            let mut requests = Vec::new();
            for a in 0..=m + 1 {
                for b in 0..=n + 1 {
                    let mut powers = Vec::new();
                    if a > 0 {
                        powers.push((x, a));
                    }
                    if b > 0 {
                        powers.push((y, b));
                    }
                    requests.push(CoefficientRequest { output: 0, powers });
                }
            }
            let result = source.coefficients(&requests, &mut budget()).unwrap();
            for xv in [-2_i64, 0, 2] {
                for yv in [0_i64, 3] {
                    let values = result
                        .program()
                        .eval_real(&[Real::from(yv), Real::from(xv)], &mut budget())
                        .unwrap();
                    assert_eq!(values[0], Real::from(xv.pow(m as u32) * yv.pow(n as u32)));
                    let mut i = 1;
                    for a in 0..=m + 1 {
                        for b in 0..=n + 1 {
                            let expected = if a > m || b > n {
                                0
                            } else {
                                choose(m, a)
                                    * choose(n, b)
                                    * xv.pow((m - a) as u32)
                                    * yv.pow((n - b) as u32)
                            };
                            assert_eq!(values[i], Real::from(expected), "m={m} n={n} a={a} b={b}");
                            i += 1;
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn high_factorials_repeated_requests_and_demanded_graphs() {
    let x = SymbolId(0);
    let source =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(x).powi(25)], &[x], &mut budget())
            .unwrap();
    let first = CoefficientRequest {
        output: 0,
        powers: vec![(x, 1)],
    };
    let low = source
        .coefficients(std::slice::from_ref(&first), &mut budget())
        .unwrap();
    let repeated = source
        .coefficients(&[first.clone(), first.clone(), first], &mut budget())
        .unwrap();
    assert_eq!(
        low.program().definition().nodes,
        repeated.program().definition().nodes
    );
    let output = &repeated.program().definition().outputs;
    assert_eq!(output[1], output[2]);
    assert_eq!(output[2], output[3]);
    let high = source
        .coefficients(
            &[
                CoefficientRequest {
                    output: 0,
                    powers: vec![(x, 25)],
                },
                CoefficientRequest {
                    output: 0,
                    powers: vec![(x, 26)],
                },
            ],
            &mut budget(),
        )
        .unwrap();
    assert!(high.program().definition().nodes.len() > low.program().definition().nodes.len());
    assert_eq!(
        high.program()
            .eval_real(&[Real::zero()], &mut budget())
            .unwrap(),
        [Real::zero(), Real::one(), Real::zero()]
    );
    assert_eq!(high.coefficient_output(0), Some(1));
    assert_eq!(high.coefficient_output(2), None);
}

#[test]
fn zero_derivatives_preserve_original_domain_guards() {
    use hypersolve::{
        SymbolRef,
        expression_program::{
            BinaryOperation, ExpressionDefinition, ExpressionNode, UnaryOperation,
        },
    };
    let x = SymbolId(0);
    let source = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![x],
            nodes: vec![
                ExpressionNode::Variable(SymbolRef { id: x, name: None }),
                ExpressionNode::Unary {
                    operation: UnaryOperation::Sqrt,
                    input: 0,
                },
                ExpressionNode::Constant(Real::zero()),
                ExpressionNode::Binary {
                    operation: BinaryOperation::Mul,
                    left: 2,
                    right: 1,
                },
            ],
            outputs: vec![3],
        },
        &mut budget(),
    )
    .unwrap();
    let coefficients = source
        .coefficients(
            &[CoefficientRequest {
                output: 0,
                powers: vec![(x, 2)],
            }],
            &mut budget(),
        )
        .unwrap();
    let original = source
        .eval_real(&[Real::from(-1)], &mut budget())
        .unwrap_err();
    let derived = coefficients
        .program()
        .eval_real(&[Real::from(-1)], &mut budget())
        .unwrap_err();
    assert_eq!(format!("{original:?}"), format!("{derived:?}"));
    let zero_power =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(x).powi(0)], &[x], &mut budget())
            .unwrap();
    let coefficients = zero_power
        .coefficients(
            &[CoefficientRequest {
                output: 0,
                powers: vec![(x, 2)],
            }],
            &mut budget(),
        )
        .unwrap();
    assert!(
        zero_power
            .eval_real(&[Real::zero()], &mut budget())
            .is_err()
    );
    assert!(
        coefficients
            .program()
            .eval_real(&[Real::zero()], &mut budget())
            .is_err()
    );
}

#[test]
fn invalid_requests_limits_cancellation_and_retry_are_explicit() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorError},
        work_budget::CancellationToken,
    };
    let x = SymbolId(0);
    let y = SymbolId(1);
    let source =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(x).powi(3)], &[x], &mut budget())
            .unwrap();
    assert!(matches!(
        source.coefficients(
            &[CoefficientRequest {
                output: 1,
                powers: vec![]
            }],
            &mut budget()
        ),
        Err(TaylorError::InvalidOutput { .. })
    ));
    for powers in [vec![(x, 0)], vec![(x, 1), (x, 1)], vec![(y, 1), (x, 1)]] {
        assert!(
            source
                .coefficients(&[CoefficientRequest { output: 0, powers }], &mut budget())
                .is_err()
        );
    }
    assert!(matches!(
        source.coefficients(
            &[CoefficientRequest {
                output: 0,
                powers: vec![(y, 1)]
            }],
            &mut budget()
        ),
        Err(TaylorError::Expression(ExpressionError::UnboundVariable(_)))
    ));
    let requests = [CoefficientRequest {
        output: 0,
        powers: vec![(x, 2)],
    }];
    let mut full = budget();
    let result = source.coefficients(&requests, &mut full).unwrap();
    let work = 10_000_000 - full.remaining_work();
    for limit in [0, work - 1] {
        assert!(matches!(
            source.coefficients(&requests, &mut ExpressionBudget::new(100_000, limit)),
            Err(TaylorError::Expression(ExpressionError::WorkExhausted))
        ));
    }
    let final_unit = source
        .coefficients(&requests, &mut ExpressionBudget::new(100_000, work))
        .unwrap();
    assert_eq!(
        final_unit.program().definition(),
        result.program().definition()
    );
    assert!(matches!(
        source.coefficients(&requests, &mut ExpressionBudget::new(1, 10_000_000)),
        Err(TaylorError::Expression(ExpressionError::SizeExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        source.coefficients(&requests, &mut cancelled),
        Err(TaylorError::Expression(ExpressionError::Cancelled))
    ));
    assert_eq!(
        source
            .coefficients(&requests, &mut budget())
            .unwrap()
            .program()
            .definition(),
        result.program().definition()
    );
    assert_eq!(
        source
            .coefficients(&[], &mut budget())
            .unwrap()
            .program()
            .definition(),
        source.definition()
    );
}
