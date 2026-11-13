#![cfg(feature = "interval-program")]

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 1_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16_384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), exponent).unwrap()
}
fn x() -> Expr {
    Expr::anonymous_symbol(SymbolId(0))
}
fn y() -> Expr {
    Expr::anonymous_symbol(SymbolId(1))
}
fn compile(source: &[Expr], variables: &[SymbolId]) -> ExpressionProgram {
    ExpressionProgram::compile(source, variables, &mut graph()).unwrap()
}

#[test]
fn every_operation_contains_an_independent_exact_value() {
    let cases = [
        (Expr::int(7), 7),
        (x(), 4),
        (x() + Expr::int(3), 7),
        (x() - Expr::int(3), 1),
        (x() * Expr::int(3), 12),
        (x() / Expr::int(2), 2),
        (-x(), -4),
        (x().powi(3), 64),
        (x().sqrt(), 2),
        (Expr::int(0).sin(), 0),
        (Expr::int(0).cos(), 1),
        (Expr::int(1).ln(), 0),
        (Expr::int(100).log10(), 2),
        (Expr::int(0).asin(), 0),
        (Expr::int(1).acos(), 0),
        (Expr::int(1).acosh(), 0),
        (Expr::int(0).atanh(), 0),
        (Expr::int(-1).powi(i64::MIN), 1),
        (Expr::int(-1).powi(i64::MAX), -1),
    ];
    let source: Vec<_> = cases.iter().map(|(expr, _)| expr.clone()).collect();
    let program = compile(&source, &[SymbolId(0)]);
    for precision in [-64, -8, 0] {
        let values = program
            .eval_dyadic(&[d(4, 4, 0)], precision, &mut graph(), &mut arithmetic())
            .unwrap();
        for (value, (_, expected)) in values.iter().zip(&cases) {
            assert!(
                value
                    .contains_point(&(*expected).into(), 0, &mut arithmetic())
                    .unwrap(),
                "{value:?} excludes {expected}"
            );
        }
    }
    // Coarse grids must not turn the log(10) denominator into a false pole.
    let logarithm = compile(&[x().log10()], &[SymbolId(0)]);
    for precision in [1, 4, 12] {
        assert!(
            logarithm
                .eval_dyadic(
                    &[d(100, 100, 0)],
                    precision,
                    &mut graph(),
                    &mut arithmetic()
                )
                .unwrap()[0]
                .contains_point(&2.into(), 0, &mut arithmetic())
                .unwrap()
        );
    }
}

#[test]
fn affine_boxes_match_independent_extrema_and_feed_further_operations() {
    let next = compile(&[-x() + Expr::int(5)], &[SymbolId(0)]);
    for a in -3_i64..=3 {
        for b in -3_i64..=3 {
            for c in [-2_i64, 0, 3] {
                let program = compile(
                    &[Expr::int(a) * x() + Expr::int(b) * y() + Expr::int(c)],
                    &[SymbolId(1), SymbolId(0)],
                );
                for (lo, hi) in [(-3_i64, 2_i64), (-1, 1), (0, 0), (1, 3)] {
                    let values = program
                        .eval_dyadic(
                            &[d(-2, 1, 0), d(lo, hi, 0)],
                            -32,
                            &mut graph(),
                            &mut arithmetic(),
                        )
                        .unwrap();
                    let lower = (a * lo).min(a * hi) + (-2 * b).min(b) + c;
                    let upper = (a * lo).max(a * hi) + (-2 * b).max(b) + c;
                    assert_eq!(values, [d(lower, upper, 0)]);
                    let continued = next
                        .eval_dyadic(&values, -32, &mut graph(), &mut arithmetic())
                        .unwrap();
                    assert_eq!(continued, [d(5 - upper, 5 - lower, 0)]);
                }
            }
        }
    }
}

#[test]
fn source_guards_survive_zero_factors_and_zero_powers() {
    // The native set convention is intentionally different from Expr's grammar.
    assert_eq!(
        d(0, 0, 0).checked_pow(0, &mut arithmetic()).unwrap(),
        d(1, 1, 0)
    );
    let power = compile(&[x().powi(0)], &[SymbolId(0)]);
    assert!(matches!(
        power.eval_dyadic(&[d(0, 0, 0)], -32, &mut graph(), &mut arithmetic()),
        Err(ExpressionError::Enclosure {
            source: IntervalError::EmptyDomain,
            ..
        })
    ));
    assert!(matches!(
        power.eval_dyadic(&[d(0, 1, 0)], -32, &mut graph(), &mut arithmetic()),
        Err(ExpressionError::DomainNotCertified { .. })
    ));
    assert_eq!(
        power
            .eval_dyadic(&[d(1, 2, 0)], -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        [d(1, 1, 0)]
    );
    let singular = Expr::Div(Box::new(Expr::int(1)), Box::new(x()));
    for source in [
        singular.clone(),
        Expr::Mul(Box::new(Expr::int(0)), Box::new(singular.clone())),
        singular.powi(0),
    ] {
        let program = compile(&[source], &[SymbolId(0)]);
        assert!(matches!(
            program.eval_dyadic(&[d(-1, 1, 0)], -32, &mut graph(), &mut arithmetic()),
            Err(ExpressionError::Enclosure {
                source: IntervalError::ContainsZero,
                ..
            })
        ));
    }
}

#[test]
fn partial_domains_and_dependency_overestimation_never_certify_full_coverage() {
    for (source, input) in [
        (x().sqrt(), d(-1, 1, 0)),
        (x().ln(), d(-1, 1, 0)),
        (x().asin(), d(-2, 1, 0)),
        (x().acos(), d(-1, 2, 0)),
        (x().acosh(), d(0, 2, 0)),
        (x().atanh(), d(-1, 0, 0)),
        (x().powi(-1), d(-1, 1, 0)),
    ] {
        let program = compile(&[source], &[SymbolId(0)]);
        assert!(matches!(
            program.eval_dyadic(&[input], -32, &mut graph(), &mut arithmetic()),
            Err(ExpressionError::DomainNotCertified { .. })
        ));
    }
    let invalid = compile(&[x().ln()], &[SymbolId(0)]);
    assert!(matches!(
        invalid.eval_dyadic(&[d(-2, -1, 0)], -32, &mut graph(), &mut arithmetic()),
        Err(ExpressionError::Enclosure {
            source: IntervalError::EmptyDomain,
            ..
        })
    ));
    // x*x+1 is positive, but independent interval multiplication loses that
    // relation. Failure is a missing domain certificate, not a false theorem
    // that the original expression is invalid on every input.
    let product = compile(
        &[(Expr::Mul(Box::new(x()), Box::new(x())) + Expr::int(1)).ln()],
        &[SymbolId(0)],
    );
    assert!(matches!(
        product.eval_dyadic(&[d(-1, 1, 0)], -32, &mut graph(), &mut arithmetic()),
        Err(ExpressionError::DomainNotCertified { .. })
    ));
    let square = compile(&[(x().powi(2) + Expr::int(1)).ln()], &[SymbolId(0)]);
    assert!(
        square
            .eval_dyadic(&[d(-1, 1, 0)], -32, &mut graph(), &mut arithmetic())
            .is_ok()
    );
}

#[test]
fn empty_products_and_binding_validation_have_distinct_semantics() {
    let program = compile(&[x(), Expr::int(2)], &[SymbolId(0)]);
    let result = program
        .eval_dyadic(
            &[DyadicInterval::empty()],
            -32,
            &mut graph(),
            &mut DyadicBudget::new(0, 0),
        )
        .unwrap();
    assert!(result.iter().all(DyadicInterval::is_empty));
    assert_eq!(result.len(), 2);
    assert!(matches!(
        program.eval_dyadic(&[], -32, &mut graph(), &mut arithmetic()),
        Err(ExpressionError::BindingCount {
            expected: 1,
            actual: 0
        })
    ));
    let constant = compile(&[Expr::int(2)], &[]);
    assert_eq!(
        constant
            .eval_dyadic(&[], -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        [d(2, 2, 0)]
    );
}

#[test]
fn both_budgets_cancellation_and_retry_are_preserved() {
    let program = compile(&[x().sqrt() + Expr::int(1)], &[SymbolId(0)]);
    let inputs = [d(1, 4, 0)];
    let expected = program
        .eval_dyadic(&inputs, -32, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(expected, [d(2, 3, 0)]);
    assert!(matches!(
        program.eval_dyadic(
            &inputs,
            -32,
            &mut ExpressionBudget::new(100_000, 0),
            &mut arithmetic()
        ),
        Err(ExpressionError::WorkExhausted)
    ));
    assert!(matches!(
        program.eval_dyadic(
            &inputs,
            -32,
            &mut graph(),
            &mut DyadicBudget::new(16_384, 0)
        ),
        Err(ExpressionError::Enclosure {
            source: IntervalError::WorkExhausted,
            ..
        })
    ));
    assert!(matches!(
        program.eval_dyadic(
            &inputs,
            -32,
            &mut graph(),
            &mut DyadicBudget::new(0, u64::MAX)
        ),
        Err(ExpressionError::Enclosure {
            source: IntervalError::SizeExhausted,
            ..
        })
    ));
    let token = CancellationToken::new();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token.clone());
    token.cancel();
    assert!(matches!(
        program.eval_dyadic(&inputs, -32, &mut cancelled, &mut arithmetic()),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(
        program
            .eval_dyadic(&inputs, -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        expected
    );
    let replay =
        ExpressionProgram::from_definition(program.definition().clone(), &mut graph()).unwrap();
    assert_eq!(
        replay
            .eval_dyadic(&inputs, -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        expected
    );
}

#[test]
fn symbolic_constants_and_existing_derivatives_keep_the_exact_source() {
    let source = x().powi(3) + Expr::real(Real::from(2).sqrt().unwrap());
    let program = compile(
        &[source.clone(), source.derivative(SymbolId(0))],
        &[SymbolId(0)],
    );
    let image = program
        .eval_dyadic(&[d(1, 2, 0)], -32, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(image[1], d(3, 12, 0));
    // 1 < sqrt(2) < 2 gives independent coarse checks for the image bounds.
    let lower = d(2, 2, 0);
    let upper = d(10, 10, 0);
    assert!(
        lower
            .strictly_left_of(&image[0], &mut arithmetic())
            .unwrap()
    );
    assert!(
        image[0]
            .strictly_left_of(&upper, &mut arithmetic())
            .unwrap()
    );
    assert_eq!(program.definition().variables, [SymbolId(0)]);
}

#[test]
fn shared_dag_outputs_and_failure_order_follow_the_source() {
    use hypersolve::expression_program::{
        BinaryOperation, ExpressionDefinition, ExpressionNode, UnaryOperation,
    };
    let mut source = ExpressionDefinition {
        variables: vec![SymbolId(0)],
        nodes: vec![
            ExpressionNode::Variable(hypersolve::SymbolRef::new(SymbolId(0), None)),
            ExpressionNode::Constant(Real::one()),
            ExpressionNode::Binary {
                operation: BinaryOperation::Add,
                left: 0,
                right: 1,
            },
            ExpressionNode::Binary {
                operation: BinaryOperation::Mul,
                left: 2,
                right: 2,
            },
            ExpressionNode::Binary {
                operation: BinaryOperation::Div,
                left: 2,
                right: 0,
            },
        ],
        outputs: vec![3, 2, 3, 4],
    };
    let program = ExpressionProgram::from_definition(source.clone(), &mut graph()).unwrap();
    let inputs = [d(1, 2, 0)];
    let expected = [d(4, 9, 0), d(2, 3, 0), d(4, 9, 0), d(1, 3, 0)];
    let mut gb = graph();
    let mut db = arithmetic();
    assert_eq!(
        program.eval_dyadic(&inputs, -32, &mut gb, &mut db).unwrap(),
        expected
    );
    let graph_work = 1_000_000 - gb.remaining_work();
    let scalar_work = 1_000_000_000_000 - db.remaining_work();
    assert_eq!(
        program
            .eval_dyadic(
                &inputs,
                -32,
                &mut ExpressionBudget::new(100_000, graph_work),
                &mut DyadicBudget::new(16_384, scalar_work)
            )
            .unwrap(),
        expected
    );
    assert!(matches!(
        program.eval_dyadic(
            &inputs,
            -32,
            &mut ExpressionBudget::new(100_000, graph_work - 1),
            &mut arithmetic()
        ),
        Err(ExpressionError::WorkExhausted)
    ));
    assert!(matches!(
        program.eval_dyadic(
            &inputs,
            -32,
            &mut graph(),
            &mut DyadicBudget::new(16_384, scalar_work - 1)
        ),
        Err(ExpressionError::Enclosure {
            source: IntervalError::WorkExhausted,
            ..
        })
    ));
    assert_eq!(
        program
            .eval_dyadic(&inputs, -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        expected
    );

    source.variables.clear();
    source.nodes = vec![
        ExpressionNode::Constant(Real::from(-1)),
        ExpressionNode::Unary {
            operation: UnaryOperation::Sqrt,
            input: 0,
        },
        ExpressionNode::Constant(Real::zero()),
        ExpressionNode::Unary {
            operation: UnaryOperation::Ln,
            input: 2,
        },
        ExpressionNode::Binary {
            operation: BinaryOperation::Add,
            left: 3,
            right: 1,
        },
    ];
    for outputs in [vec![3, 1], vec![4]] {
        source.outputs = outputs;
        let program = ExpressionProgram::from_definition(source.clone(), &mut graph()).unwrap();
        assert!(matches!(
            program.eval_dyadic(&[], -32, &mut graph(), &mut arithmetic()),
            Err(ExpressionError::Enclosure {
                node: 3,
                source: IntervalError::EmptyDomain
            })
        ));
    }
    source.outputs = vec![2];
    let program = ExpressionProgram::from_definition(source, &mut graph()).unwrap();
    assert_eq!(
        program
            .eval_dyadic(&[], -32, &mut graph(), &mut arithmetic())
            .unwrap(),
        [d(0, 0, 0)]
    );
}
