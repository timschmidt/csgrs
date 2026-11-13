#![cfg(feature = "float-proposal-program")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
        ExpressionProgram, FloatProposalError, UnaryOperation,
    },
    work_budget::CancellationToken,
};
const X: SymbolId = SymbolId(0);
const T: SymbolId = SymbolId(1);
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(100000, 10000000)
}
fn program(outputs: &[Expr]) -> ExpressionProgram {
    ExpressionProgram::compile(outputs, &[T, X], &mut budget()).unwrap()
}
fn x() -> Expr {
    Expr::anonymous_symbol(X)
}
#[test]
fn primitive_estimates_match_independent_exact_polynomial_controls() {
    let t = Expr::anonymous_symbol(T);
    let source = program(&[
        x().powi(3) + Expr::int(2) * t.clone() - Expr::int(3),
        (x() + t) / (x().powi(2) + Expr::int(1)),
    ]);
    let prepared = source.float_proposal(&mut budget()).unwrap();
    assert_eq!(prepared.source().definition(), source.definition());
    for a in -8_i64..=8 {
        for b in -8_i64..=8 {
            assert_eq!(
                prepared
                    .estimate(&[a as f64, b as f64], &mut budget())
                    .unwrap(),
                [
                    (b * b * b + 2 * a - 3) as f64,
                    (a + b) as f64 / (b * b + 1) as f64
                ]
            );
        }
    }
    let source = program(&[
        -x(),
        x().sqrt(),
        x().sin(),
        x().cos(),
        (x() + Expr::int(1)).ln(),
        (x() + Expr::int(1)).log10(),
        x().asin(),
        (x() + Expr::int(1)).acos(),
        (x() + Expr::int(1)).acosh(),
        x().atanh(),
    ]);
    assert_eq!(
        source
            .float_proposal(&mut budget())
            .unwrap()
            .estimate(&[0.0, 0.0], &mut budget())
            .unwrap(),
        [-0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    );
}

#[test]
fn full_width_integer_powers_preserve_parity_domains_and_bounded_work() {
    for exponent in [
        i64::MIN,
        i64::MIN + 1,
        -1,
        0,
        1,
        (1_i64 << 53) + 1,
        i64::MAX,
    ] {
        let prepared = program(&[x().powi(exponent)])
            .float_proposal(&mut budget())
            .unwrap();
        let expected = if exponent & 1 == 0 { 1.0 } else { -1.0 };
        assert_eq!(
            prepared.estimate(&[0.0, -1.0], &mut budget()).unwrap(),
            [expected]
        );
        assert_eq!(
            prepared.estimate(&[0.0, 1.0], &mut budget()).unwrap(),
            [1.0]
        );
        if exponent <= 0 {
            assert!(matches!(
                prepared.estimate(&[0.0, 0.0], &mut budget()),
                Err(FloatProposalError::NativeDomain { .. })
            ));
        }
    }
    let negative = program(&[x().powi(-2)])
        .float_proposal(&mut budget())
        .unwrap();
    assert_eq!(
        negative.estimate(&[0.0, 1e200], &mut budget()).unwrap(),
        [0.0]
    );
    assert!(matches!(
        negative.estimate(&[0.0, 1e-200], &mut budget()),
        Err(FloatProposalError::NonFiniteResult { .. })
    ));
    let positive = program(&[x().powi(2)])
        .float_proposal(&mut budget())
        .unwrap();
    assert!(matches!(
        positive.estimate(&[0.0, f64::MAX], &mut budget()),
        Err(FloatProposalError::NonFiniteResult { .. })
    ));
}

#[test]
fn shared_nodes_are_evaluated_once_and_unused_guards_are_not_demanded() {
    let source = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![X],
            nodes: vec![
                ExpressionNode::Variable(SymbolRef::new(X, None)),
                ExpressionNode::Unary {
                    operation: UnaryOperation::PowI(2),
                    input: 0,
                },
                ExpressionNode::Binary {
                    operation: BinaryOperation::Add,
                    left: 1,
                    right: 1,
                },
                ExpressionNode::Constant(Real::from(-1)),
                ExpressionNode::Unary {
                    operation: UnaryOperation::Sqrt,
                    input: 3,
                },
            ],
            outputs: vec![2, 2],
        },
        &mut budget(),
    )
    .unwrap();
    let prepared = source.float_proposal(&mut budget()).unwrap();
    // 1 argument + 5 initialized entries + 3 demanded nodes + 4 power-loop units + 2 outputs.
    let mut exact = ExpressionBudget::new(100, 15);
    assert_eq!(prepared.estimate(&[3.0], &mut exact).unwrap(), [18.0, 18.0]);
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        prepared.estimate(&[3.0], &mut ExpressionBudget::new(100, 14)),
        Err(FloatProposalError::Expression(
            ExpressionError::WorkExhausted
        ))
    ));
    assert_eq!(
        prepared.estimate(&[3.0], &mut budget()).unwrap(),
        [18.0, 18.0]
    );
}

#[test]
fn guards_nonfinite_inputs_projection_failure_and_retry_stay_explicit() {
    for expression in [
        Expr::int(0) * (Expr::int(1) / x()),
        (Expr::int(1) / x()).powi(0),
    ] {
        let prepared = program(&[expression])
            .float_proposal(&mut budget())
            .unwrap();
        assert!(matches!(
            prepared.estimate(&[0.0, 0.0], &mut budget()),
            Err(FloatProposalError::NativeDomain { .. })
        ));
    }
    for (expression, value) in [
        (x().sqrt(), -1.0),
        (x().ln(), 0.0),
        (x().log10(), -1.0),
        (x().asin(), 2.0),
        (x().acos(), -2.0),
        (x().acosh(), 0.0),
        (x().atanh(), 1.0),
    ] {
        let prepared = program(&[expression])
            .float_proposal(&mut budget())
            .unwrap();
        assert!(matches!(
            prepared.estimate(&[0.0, value], &mut budget()),
            Err(FloatProposalError::NativeDomain { .. })
        ));
    }
    let source = program(&[x() + Expr::int(1)]);
    let prepared = source.float_proposal(&mut budget()).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            prepared.estimate(&[value, 1.0], &mut budget()),
            Err(FloatProposalError::NonFiniteArgument { slot: 0 })
        ));
    }
    assert!(matches!(
        prepared.estimate(&[], &mut budget()),
        Err(FloatProposalError::Expression(
            ExpressionError::BindingCount { .. }
        ))
    ));
    let huge = Real::from(2).powi(2048.into()).unwrap();
    assert!(matches!(
        program(&[Expr::Constant(huge)]).float_proposal(&mut budget()),
        Err(FloatProposalError::ConstantUnavailable { .. })
    ));
    let mut measured = budget();
    source.float_proposal(&mut measured).unwrap();
    let used = budget().remaining_work() - measured.remaining_work();
    let mut exact = ExpressionBudget::new(100000, used);
    source.float_proposal(&mut exact).unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        source.float_proposal(&mut ExpressionBudget::new(100000, used - 1)),
        Err(FloatProposalError::Expression(
            ExpressionError::WorkExhausted
        ))
    ));
    assert!(matches!(
        source.float_proposal(&mut ExpressionBudget::new(1, 100000)),
        Err(FloatProposalError::Expression(
            ExpressionError::SizeExhausted
        ))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        source.float_proposal(&mut cancelled),
        Err(FloatProposalError::Expression(ExpressionError::Cancelled))
    ));
    assert!(matches!(
        prepared.estimate(&[0.0, 1.0], &mut cancelled),
        Err(FloatProposalError::Expression(ExpressionError::Cancelled))
    ));
    assert_eq!(
        source
            .float_proposal(&mut budget())
            .unwrap()
            .estimate(&[0.0, 1.0], &mut budget())
            .unwrap(),
        [2.0]
    );
}
