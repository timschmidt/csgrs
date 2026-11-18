#![cfg(feature = "expression-program")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, CompositionError, ExpressionBudget, ExpressionDefinition, ExpressionError,
        ExpressionNode, ExpressionProgram,
    },
    work_budget::CancellationToken,
};
const X: SymbolId = SymbolId(5);
const Y: SymbolId = SymbolId(11);
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn compile(outputs: &[Expr], bindings: &[SymbolId]) -> ExpressionProgram {
    ExpressionProgram::compile(outputs, bindings, &mut budget()).unwrap()
}
fn pair() -> (ExpressionProgram, ExpressionProgram) {
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    (
        compile(&[y.clone().powi(2) + Expr::int(3) * x.clone()], &[Y, X]),
        compile(&[x.clone() + y.clone(), x - y], &[Y, X]),
    )
}

#[test]
fn independent_scopes_binding_permutation_and_chain_rule_are_exact() {
    let (outer, inner) = pair();
    let composed = outer.compose(&inner, &[1, 0], &mut budget()).unwrap();
    assert_eq!(composed.outer().definition(), outer.definition());
    assert_eq!(composed.inner().definition(), inner.definition());
    assert_eq!(composed.bindings(), &[1, 0]);
    assert_eq!(composed.program().definition().variables, vec![Y, X]);
    assert_eq!(composed.composed_output(0), Some(2));
    assert_eq!(composed.composed_output(1), None);
    assert_eq!(composed.composed_output(usize::MAX), None);
    let jacobian = composed.program().jacobian(&[X, Y], &mut budget()).unwrap();
    for x in -3_i64..=3 {
        for y in -3_i64..=3 {
            let values = jacobian
                .program()
                .eval_real(&[y.into(), x.into()], &mut budget())
                .unwrap();
            assert_eq!(values[0], Real::from(x + y));
            assert_eq!(values[1], Real::from(x - y));
            assert_eq!(values[2], Real::from((x - y).pow(2) + 3 * (x + y)));
            assert_eq!(
                values[jacobian.partial_output(2, 0).unwrap()],
                Real::from(2 * (x - y) + 3)
            );
            assert_eq!(
                values[jacobian.partial_output(2, 1).unwrap()],
                Real::from(-2 * (x - y) + 3)
            );
        }
    }
    let z = Expr::anonymous_symbol(X);
    let next = compile(&[z.clone().powi(2) + z], &[X]);
    let twice = next
        .compose(composed.program(), &[2], &mut budget())
        .unwrap();
    assert_eq!(twice.composed_output(0), Some(3));
    for x in -3_i64..=3 {
        let f = x * x + 3 * x;
        let values = twice
            .program()
            .eval_real(&[0.into(), x.into()], &mut budget())
            .unwrap();
        assert_eq!(values[3], Real::from(f * f + f));
    }
}

#[test]
fn inner_output_guards_survive_ignored_bindings_and_empty_outer_outputs() {
    let x = Expr::anonymous_symbol(X);
    let inner = compile(&[Expr::int(1) / x], &[X]);
    for outer in [compile(&[Expr::int(0)], &[]), compile(&[], &[])] {
        let composed = outer.compose(&inner, &[], &mut budget()).unwrap();
        assert_eq!(
            composed.program().definition().outputs[0],
            inner.definition().outputs[0]
        );
        assert!(
            matches!(composed.program().eval_real(&[Real::zero()], &mut budget()), Err(ExpressionError::Scalar { node, .. }) if node == inner.definition().outputs[0])
        );
        let values = composed
            .program()
            .eval_real(&[Real::one()], &mut budget())
            .unwrap();
        assert_eq!(values[0], Real::one());
        assert_eq!(values.len(), 1 + outer.definition().outputs.len());
    }
    let outer = compile(
        &[Expr::int(0) * (Expr::int(1) / Expr::anonymous_symbol(Y))],
        &[Y],
    );
    let inner = compile(&[Expr::anonymous_symbol(X)], &[X]);
    let composed = outer.compose(&inner, &[0], &mut budget()).unwrap();
    let division = outer
        .definition()
        .nodes
        .iter()
        .position(|node| {
            matches!(
                node,
                ExpressionNode::Binary {
                    operation: BinaryOperation::Div,
                    ..
                }
            )
        })
        .unwrap();
    assert!(
        matches!(composed.program().eval_real(&[Real::zero()], &mut budget()), Err(ExpressionError::Scalar { node, .. }) if Some(node) == composed.outer_node(division))
    );
    // An unreachable invalid node remains data, as in the original program.
    let original = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![],
            nodes: vec![
                ExpressionNode::Constant(Real::zero()),
                ExpressionNode::Binary {
                    operation: BinaryOperation::Div,
                    left: 0,
                    right: 0,
                },
            ],
            outputs: vec![0],
        },
        &mut budget(),
    )
    .unwrap();
    let selected = compile(&[Expr::anonymous_symbol(Y)], &[Y])
        .compose(&original, &[0], &mut budget())
        .unwrap();
    assert_eq!(
        selected.program().eval_real(&[], &mut budget()).unwrap(),
        vec![Real::zero(), Real::zero()]
    );
    assert_eq!(
        &selected.program().definition().nodes[..2],
        original.definition().nodes
    );
}

#[test]
fn repeated_references_keep_linear_graphs_and_source_node_mappings() {
    let mut nodes = vec![
        ExpressionNode::Variable(SymbolRef::new(Y, None)),
        ExpressionNode::Variable(SymbolRef::new(Y, None)),
    ];
    nodes.push(ExpressionNode::Binary {
        operation: BinaryOperation::Add,
        left: 0,
        right: 1,
    });
    for _ in 0..128 {
        let index = nodes.len() - 1;
        nodes.push(ExpressionNode::Binary {
            operation: BinaryOperation::Mul,
            left: index,
            right: index,
        });
    }
    let root = nodes.len() - 1;
    let outer = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![Y],
            nodes,
            outputs: vec![root, root],
        },
        &mut budget(),
    )
    .unwrap();
    let inner = compile(&[Expr::anonymous_symbol(X).powi(2)], &[X]);
    let composed = outer.compose(&inner, &[0], &mut budget()).unwrap();
    assert_eq!(composed.outer_node(0), composed.outer_node(1));
    assert_eq!(composed.outer_node(root + 1), None);
    assert_eq!(
        composed.program().definition().nodes.len(),
        inner.definition().nodes.len() + outer.definition().nodes.len() - 2
    );
    assert_eq!(
        composed
            .program()
            .eval_real(&[Real::zero()], &mut budget())
            .unwrap(),
        vec![Real::zero(); 3]
    );
    let duplicate = compile(
        &[Expr::anonymous_symbol(Y) - Expr::anonymous_symbol(X)],
        &[Y, X],
    )
    .compose(&inner, &[0, 0], &mut budget())
    .unwrap();
    assert_eq!(
        duplicate
            .program()
            .eval_real(&[Real::from(3)], &mut budget())
            .unwrap(),
        vec![Real::from(9), Real::zero()]
    );
}

#[test]
fn shape_limits_cancellation_and_retry_are_explicit() {
    let (outer, inner) = pair();
    assert!(matches!(
        outer.compose(&inner, &[0], &mut budget()),
        Err(CompositionError::BindingCount {
            expected: 2,
            actual: 1
        })
    ));
    assert!(matches!(
        outer.compose(&inner, &[2, 0], &mut budget()),
        Err(CompositionError::InvalidInput {
            binding: 0,
            output: 2
        })
    ));
    let mut measured = budget();
    let expected = outer.compose(&inner, &[1, 0], &mut measured).unwrap();
    let used = budget().remaining_work() - measured.remaining_work();
    let mut exact = ExpressionBudget::new(1_000_000, used);
    assert_eq!(
        outer
            .compose(&inner, &[1, 0], &mut exact)
            .unwrap()
            .program()
            .definition(),
        expected.program().definition()
    );
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        outer.compose(
            &inner,
            &[1, 0],
            &mut ExpressionBudget::new(1_000_000, used - 1)
        ),
        Err(CompositionError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(matches!(
        outer.compose(&inner, &[1, 0], &mut ExpressionBudget::new(1, 100_000_000)),
        Err(CompositionError::Expression(ExpressionError::SizeExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    let error = outer.compose(&inner, &[1, 0], &mut cancelled).unwrap_err();
    assert!(matches!(
        error,
        CompositionError::Expression(ExpressionError::Cancelled)
    ));
    assert!(std::error::Error::source(&error).is_some());
    assert_eq!(
        outer
            .compose(&inner, &[1, 0], &mut budget())
            .unwrap()
            .program()
            .definition(),
        expected.program().definition()
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn composition_construction_performs_no_scalar_dispatch_or_predicate_queries() {
    use hyperreal::dispatch_trace;
    let (outer, inner) = pair();
    dispatch_trace::reset();
    let composed =
        dispatch_trace::with_recording(|| outer.compose(&inner, &[1, 0], &mut budget()).unwrap());
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
    assert!(std::ptr::eq(
        composed.outer().definition(),
        outer.definition()
    ));
    assert!(std::ptr::eq(
        composed.inner().definition(),
        inner.definition()
    ));
}

#[cfg(feature = "serde")]
#[test]
fn replay_revalidates_both_original_sources_and_the_binding_selection() {
    if let Ok(data) = std::env::var("HYPERSOLVE_COMPOSITION_REPLAY") {
        let (outer, inner, bindings): (ExpressionDefinition, ExpressionDefinition, Vec<usize>) =
            serde_json::from_str(&data).unwrap();
        let outer = ExpressionProgram::from_definition(outer, &mut budget()).unwrap();
        let inner = ExpressionProgram::from_definition(inner, &mut budget()).unwrap();
        let replay = outer.compose(&inner, &bindings, &mut budget()).unwrap();
        assert_eq!(
            replay
                .program()
                .eval_real(&[2.into(), 3.into()], &mut budget())
                .unwrap(),
            vec![Real::from(5), Real::from(1), Real::from(16)]
        );
        return;
    }
    let (outer, inner) = pair();
    let composed = outer.compose(&inner, &[1, 0], &mut budget()).unwrap();
    let data =
        serde_json::to_vec(&(outer.definition(), inner.definition(), composed.bindings())).unwrap();
    let (outer, inner, bindings): (ExpressionDefinition, ExpressionDefinition, Vec<usize>) =
        serde_json::from_slice(&data).unwrap();
    let outer = ExpressionProgram::from_definition(outer, &mut budget()).unwrap();
    let inner = ExpressionProgram::from_definition(inner, &mut budget()).unwrap();
    let replay = outer.compose(&inner, &bindings, &mut budget()).unwrap();
    assert_eq!(
        replay.program().definition(),
        composed.program().definition()
    );
    assert!(
        outer
            .compose(&inner, &[usize::MAX, 0], &mut budget())
            .is_err()
    );
    let mut invalid = outer.definition().clone();
    invalid.outputs[0] = usize::MAX;
    assert!(matches!(
        ExpressionProgram::from_definition(invalid, &mut budget()),
        Err(ExpressionError::InvalidOutput { .. })
    ));
    let child = std::process::Command::new(replay_binary::path().unwrap())
        .args([
            "--exact",
            "replay_revalidates_both_original_sources_and_the_binding_selection",
            "--nocapture",
        ])
        .env(
            "HYPERSOLVE_COMPOSITION_REPLAY",
            std::str::from_utf8(&data).unwrap(),
        )
        .output()
        .unwrap();
    assert!(
        child.status.success(),
        "{}",
        String::from_utf8_lossy(&child.stderr)
    );
}

#[cfg(feature = "interval-program")]
#[test]
fn zero_input_unit_and_empty_products_keep_original_domain_semantics() {
    use hyperinterval::{DyadicBudget, DyadicInterval};
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000);
    let unit = compile(&[], &[]);
    let constant = compile(&[Expr::int(7)], &[]);
    let composed = constant.compose(&unit, &[], &mut budget()).unwrap();
    assert_eq!(composed.composed_output(0), Some(0));
    assert_eq!(
        composed
            .program()
            .eval_dyadic(&[], -64, &mut budget(), &mut arithmetic)
            .unwrap(),
        vec![DyadicInterval::point(7.into(), 0).unwrap()]
    );
    let none = unit.compose(&constant, &[], &mut budget()).unwrap();
    assert_eq!(none.composed_output(0), None);
    assert_eq!(
        none.program().eval_real(&[], &mut budget()).unwrap(),
        vec![Real::from(7)]
    );
    let inner = compile(&[Expr::anonymous_symbol(X)], &[X]);
    let composed = constant.compose(&inner, &[], &mut budget()).unwrap();
    assert_eq!(
        composed
            .program()
            .eval_dyadic(
                &[DyadicInterval::empty()],
                -64,
                &mut budget(),
                &mut arithmetic
            )
            .unwrap(),
        vec![DyadicInterval::empty(); 2]
    );
    assert!(matches!(
        composed
            .program()
            .eval_dyadic(&[], -64, &mut budget(), &mut arithmetic),
        Err(ExpressionError::BindingCount {
            expected: 1,
            actual: 0
        })
    ));
}

#[cfg(feature = "taylor-affine")]
#[test]
fn nonlinear_taylor_projection_retains_correlation_and_all_source_guards() {
    use hyperinterval::{DyadicBudget, DyadicInterval};
    use hypersolve::expression_program::{
        TaylorAffineMap, TaylorAffineModel, TaylorAffineRow, TaylorRequest,
    };
    use std::sync::Arc;
    let d = |a: i64, b: i64, e: i32| DyadicInterval::new(a.into(), b.into(), e).unwrap();
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
    let x = Expr::anonymous_symbol(X);
    let inner = compile(&[x.clone(), Expr::int(1) - x], &[X]);
    let outer = compile(
        &[Expr::anonymous_symbol(X) * Expr::anonymous_symbol(Y)],
        &[X, Y],
    );
    let composed = outer.compose(&inner, &[0, 1], &mut budget()).unwrap();
    let natural = composed
        .program()
        .eval_dyadic(&[d(0, 1, 0)], -64, &mut budget(), &mut arithmetic)
        .unwrap();
    assert_eq!(natural[2], d(0, 1, 0));
    let model = Arc::new(
        composed
            .program()
            .taylor_model(
                TaylorRequest {
                    center: vec![d(1, 1, -1)],
                    domain: vec![d(0, 1, 0)],
                    order: 2,
                    precision: -64,
                },
                &mut budget(),
                &mut arithmetic,
            )
            .unwrap(),
    );
    assert_eq!(model.image()[2], d(0, 1, -2));
    assert_eq!(model.remainder(), vec![d(0, 0, 0); 3]);
    let projection = TaylorAffineModel::new(
        model.clone(),
        TaylorAffineMap {
            input_count: 3,
            rows: vec![TaylorAffineRow {
                coefficients: vec![Real::zero(), Real::zero(), Real::one()],
                offset: Real::zero(),
            }],
        },
        &mut budget(),
        &mut arithmetic,
    )
    .unwrap();
    assert!(Arc::ptr_eq(projection.source(), &model));
    assert_eq!(projection.range().bounds(), &[d(0, 1, -2)]);
    assert_eq!(
        projection
            .enclose_on(&[d(1, 1, -1)], &mut budget(), &mut arithmetic)
            .unwrap()
            .bounds(),
        &[d(1, 1, -2)]
    );
    assert!(
        projection
            .enclose_on(&[d(2, 2, 0)], &mut budget(), &mut arithmetic)
            .is_err()
    );
    assert!(
        projection
            .enclose_on(&[DyadicInterval::empty()], &mut budget(), &mut arithmetic)
            .unwrap()
            .is_empty()
    );
    let bad = compile(&[Expr::int(1) / Expr::anonymous_symbol(X)], &[X]);
    let zero = compile(&[Expr::int(0)], &[])
        .compose(&bad, &[], &mut budget())
        .unwrap();
    assert!(
        zero.program()
            .eval_dyadic(&[d(0, 0, 0)], -64, &mut budget(), &mut arithmetic)
            .is_err()
    );
    assert_eq!(
        zero.program()
            .eval_dyadic(
                &[DyadicInterval::empty()],
                -64,
                &mut budget(),
                &mut arithmetic
            )
            .unwrap(),
        vec![DyadicInterval::empty(); 2]
    );
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
