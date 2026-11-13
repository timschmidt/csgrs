#![cfg(feature = "expression-program")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId, SymbolRef,
    expression_program::{
        AffineParameterization, BinaryOperation, ExpressionBudget, ExpressionDefinition,
        ExpressionError, ExpressionNode, ExpressionProgram,
    },
    work_budget::CancellationToken,
};
fn b() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn spec(scale: i64, offset: i64) -> AffineParameterization {
    AffineParameterization {
        variable: SymbolId(0),
        scale: scale.into(),
        offset: offset.into(),
    }
}
fn source() -> ExpressionProgram {
    let t = Expr::anonymous_symbol(SymbolId(0));
    let x = Expr::anonymous_symbol(SymbolId(1));
    ExpressionProgram::compile(
        &[t.clone().powi(2) + x.clone(), t * x],
        &[SymbolId(1), SymbolId(0)],
        &mut b(),
    )
    .unwrap()
}
#[test]
fn exact_affine_time_outputs_and_partials_match_integer_controls() {
    let original = source();
    for scale in -3..=3 {
        for offset in -3..=3 {
            let change = original
                .reparameterize(spec(scale, offset), &mut b())
                .unwrap();
            assert_eq!(change.source().definition(), original.definition());
            assert_eq!(
                change.program().definition().variables,
                original.definition().variables
            );
            let jac = change
                .program()
                .jacobian(&[SymbolId(0), SymbolId(1)], &mut b())
                .unwrap();
            for t in -3_i64..=3 {
                for x in -3_i64..=3 {
                    let old = offset + scale * t;
                    let values = jac
                        .program()
                        .eval_real(&[x.into(), t.into()], &mut b())
                        .unwrap();
                    assert_eq!(
                        values,
                        [
                            scale * (old * old + x),
                            scale * old * x,
                            2 * scale * scale * old,
                            scale * scale * x,
                            scale,
                            scale * old
                        ]
                        .map(Real::from)
                    );
                }
            }
        }
    }
}
#[test]
fn repeated_variable_nodes_and_shared_depth_preserve_linear_graphs() {
    let mut nodes = vec![
        ExpressionNode::Variable(SymbolRef::new(SymbolId(0), None)),
        ExpressionNode::Variable(SymbolRef::new(SymbolId(0), None)),
    ];
    nodes.push(ExpressionNode::Binary {
        operation: BinaryOperation::Add,
        left: 0,
        right: 1,
    });
    for _ in 0..128 {
        let input = nodes.len() - 1;
        nodes.push(ExpressionNode::Binary {
            operation: BinaryOperation::Mul,
            left: input,
            right: input,
        });
    }
    let root = nodes.len() - 1;
    let original = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![SymbolId(0)],
            nodes,
            outputs: vec![root, root],
        },
        &mut b(),
    )
    .unwrap();
    let change = original.reparameterize(spec(1, 0), &mut b()).unwrap();
    assert_eq!(change.transformed_node(0), change.transformed_node(1));
    assert_eq!(change.transformed_node(root + 1), None);
    assert!(change.program().definition().nodes.len() <= original.definition().nodes.len() + 6);
    assert_eq!(
        change
            .program()
            .eval_real(&[Real::zero()], &mut b())
            .unwrap(),
        [Real::zero(), Real::zero()]
    );
}
#[test]
fn source_guards_survive_zero_scale_and_exhaustion_does_not_poison_retry() {
    let t = Expr::anonymous_symbol(SymbolId(0));
    let original =
        ExpressionProgram::compile(&[Expr::int(1) / t], &[SymbolId(0)], &mut b()).unwrap();
    let change = original.reparameterize(spec(0, 0), &mut b()).unwrap();
    assert!(change.program().eval_real(&[1.into()], &mut b()).is_err());
    #[cfg(feature = "interval-program")]
    {
        assert!(
            change
                .program()
                .eval_dyadic(
                    &[hyperinterval::DyadicInterval::point(1.into(), 0).unwrap()],
                    -64,
                    &mut b(),
                    &mut hyperinterval::DyadicBudget::new(16384, 1_000_000_000)
                )
                .is_err()
        );
    }
    let original = source();
    let mut measured = b();
    let expected = original.reparameterize(spec(2, 3), &mut measured).unwrap();
    let used = b().remaining_work() - measured.remaining_work();
    let mut exact = ExpressionBudget::new(1_000_000, used);
    assert_eq!(
        original
            .reparameterize(spec(2, 3), &mut exact)
            .unwrap()
            .program()
            .definition(),
        expected.program().definition()
    );
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        original.reparameterize(spec(2, 3), &mut ExpressionBudget::new(1_000_000, used - 1)),
        Err(ExpressionError::WorkExhausted)
    ));
    assert!(matches!(
        original.reparameterize(spec(2, 3), &mut ExpressionBudget::new(1, 100_000_000)),
        Err(ExpressionError::SizeExhausted)
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = b();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        original.reparameterize(spec(2, 3), &mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(
        original
            .reparameterize(spec(2, 3), &mut b())
            .unwrap()
            .program()
            .definition(),
        expected.program().definition()
    );
    let mut invalid = spec(1, 0);
    invalid.variable = SymbolId(999);
    assert!(matches!(
        original.reparameterize(invalid, &mut b()),
        Err(ExpressionError::UnboundVariable(SymbolId(999)))
    ));
}
