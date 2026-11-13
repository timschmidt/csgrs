#![cfg(feature = "expression-program")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
        ExpressionProgram, UnaryOperation,
    },
    work_budget::CancellationToken,
};
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn x() -> Expr {
    Expr::anonymous_symbol(SymbolId(0))
}
fn compile(source: &[Expr], variables: &[SymbolId]) -> ExpressionProgram {
    ExpressionProgram::compile(source, variables, &mut budget()).unwrap()
}

#[test]
fn polynomial_partials_have_independent_integer_expectations_and_compose() {
    let y = || Expr::anonymous_symbol(SymbolId(1));
    let source = compile(
        &[x().powi(3) + Expr::int(2) * x() * y(), y().powi(2) - x()],
        &[SymbolId(1), SymbolId(0)],
    );
    let jacobian = source
        .jacobian(&[SymbolId(0), SymbolId(1)], &mut budget())
        .unwrap();
    assert_eq!(jacobian.source().definition(), source.definition());
    assert_eq!(jacobian.variables(), [SymbolId(0), SymbolId(1)]);
    assert_eq!(jacobian.partial_output(2, 0), None);
    assert_eq!(jacobian.partial_output(0, 2), None);
    for a in -4_i64..=4 {
        for b in -4_i64..=4 {
            let values = jacobian
                .program()
                .eval_real(&[b.into(), a.into()], &mut budget())
                .unwrap();
            let expected = [
                a * a * a + 2 * a * b,
                b * b - a,
                3 * a * a + 2 * b,
                -1,
                2 * a,
                2 * b,
            ];
            assert_eq!(values, expected.map(Real::from));
        }
    }
    let second = jacobian
        .program()
        .jacobian(&[SymbolId(0)], &mut budget())
        .unwrap();
    let values = second
        .program()
        .eval_real(&[3.into(), 2.into()], &mut budget())
        .unwrap();
    assert_eq!(
        values[second
            .partial_output(jacobian.partial_output(0, 0).unwrap(), 0)
            .unwrap()],
        Real::from(12)
    );
    assert_eq!(
        values[second
            .partial_output(jacobian.partial_output(0, 1).unwrap(), 0)
            .unwrap()],
        Real::from(2)
    );
}

#[test]
fn all_operator_rules_and_extreme_powers_have_exact_partial_values() {
    let cases = vec![
        (Expr::int(7), Real::zero()),
        (x(), Real::one()),
        (x() + Expr::int(2), Real::one()),
        (x() - Expr::int(2), Real::one()),
        (x() * Expr::int(3), Real::from(3)),
        (x() / Expr::int(2), (Real::one() / Real::from(2)).unwrap()),
        (-x(), Real::from(-1)),
        ((x() + Expr::int(1)).powi(3), Real::from(3)),
        (
            (x() + Expr::int(4)).sqrt(),
            (Real::one() / Real::from(4)).unwrap(),
        ),
        (x().sin(), Real::one()),
        (x().cos(), Real::zero()),
        ((x() + Expr::int(1)).ln(), Real::one()),
        (
            (x() + Expr::int(1)).log10(),
            (Real::one() / Real::from(10).ln().unwrap()).unwrap(),
        ),
        (x().asin(), Real::one()),
        (x().acos(), Real::from(-1)),
        (
            (x() + Expr::real((Real::from(5) / Real::from(4)).unwrap())).acosh(),
            (Real::from(4) / Real::from(3)).unwrap(),
        ),
        (x().atanh(), Real::one()),
        ((x() + Expr::int(1)).powi(i64::MIN), Real::from(i64::MIN)),
        ((x() + Expr::int(1)).powi(i64::MAX), Real::from(i64::MAX)),
    ];
    for (source, expected) in cases {
        let program = compile(&[source], &[SymbolId(0)]);
        let jacobian = program.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
        let values = jacobian
            .program()
            .eval_real(&[Real::zero()], &mut budget())
            .unwrap();
        assert_eq!(values[1], expected, "{:?}", program.definition());
    }
}

#[test]
fn original_domain_guards_survive_identically_zero_partials() {
    for (source, excluded) in [
        (x().powi(0), 0),
        (Expr::int(0) * x().ln(), -1),
        (x().sqrt(), -1),
    ] {
        let program = compile(&[source], &[SymbolId(0), SymbolId(1)]);
        let jacobian = program.jacobian(&[SymbolId(1)], &mut budget()).unwrap();
        assert_eq!(
            jacobian
                .program()
                .eval_real(&[1.into(), 0.into()], &mut budget())
                .unwrap()[1],
            Real::zero()
        );
        assert!(matches!(
            jacobian
                .program()
                .eval_real(&[excluded.into(), 0.into()], &mut budget()),
            Err(ExpressionError::Scalar { .. })
        ));
    }
    let program = compile(&[x().powi(0)], &[SymbolId(0)]);
    let jacobian = program.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
    assert!(matches!(
        jacobian.program().eval_real(&[Real::zero()], &mut budget()),
        Err(ExpressionError::Scalar { .. })
    ));
    for source in [
        Expr::int(0).sqrt(),
        Expr::int(1).acos(),
        Expr::int(1).acosh(),
    ] {
        let jacobian = compile(&[source], &[SymbolId(0)])
            .jacobian(&[SymbolId(0)], &mut budget())
            .unwrap();
        assert_eq!(
            jacobian
                .program()
                .eval_real(&[Real::zero()], &mut budget())
                .unwrap(),
            [Real::zero(), Real::zero()]
        );
    }
    let jacobian = compile(&[Expr::int(-1).sqrt()], &[SymbolId(0)])
        .jacobian(&[SymbolId(0)], &mut budget())
        .unwrap();
    assert!(matches!(
        jacobian.program().eval_real(&[Real::zero()], &mut budget()),
        Err(ExpressionError::Scalar { .. })
    ));
}

#[test]
fn shared_source_graphs_expand_linearly_and_deep_graphs_use_a_small_stack() {
    std::thread::Builder::new()
        .stack_size(128 * 1024)
        .spawn(|| {
            let mut definition = ExpressionDefinition {
                variables: vec![SymbolId(0)],
                nodes: vec![ExpressionNode::Variable(SymbolRef::new(SymbolId(0), None))],
                outputs: vec![],
            };
            for index in 0..128 {
                definition.nodes.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Mul,
                    left: index,
                    right: index,
                });
            }
            definition.outputs = vec![128, 128];
            let source = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
            let jacobian = source.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
            assert!(
                jacobian.program().definition().nodes.len() <= 5 * source.definition().nodes.len()
            );
            let slope = Real::from(2).powi_i64(128).unwrap();
            assert_eq!(
                jacobian
                    .program()
                    .eval_real(&[Real::one()], &mut budget())
                    .unwrap(),
                [Real::one(), Real::one(), slope.clone(), slope]
            );
            let mut definition = ExpressionDefinition {
                variables: vec![SymbolId(0)],
                nodes: vec![ExpressionNode::Variable(SymbolRef::new(SymbolId(0), None))],
                outputs: vec![20_000],
            };
            for input in 0..20_000 {
                definition.nodes.push(ExpressionNode::Unary {
                    operation: UnaryOperation::Neg,
                    input,
                });
            }
            let source = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
            let jacobian = source.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
            assert!(
                jacobian.program().definition().nodes.len() <= 3 * source.definition().nodes.len()
            );
            assert_eq!(
                jacobian
                    .program()
                    .eval_real(&[17.into()], &mut budget())
                    .unwrap(),
                [17.into(), Real::one()]
            );
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn bindings_empty_columns_limits_cancellation_and_replay_are_checked() {
    let source = compile(&[x().sin() + x().powi(3)], &[SymbolId(0)]);
    let mut work = budget();
    let jacobian = source.jacobian(&[SymbolId(0)], &mut work).unwrap();
    let consumed = 100_000_000 - work.remaining_work();
    let exact = source
        .jacobian(
            &[SymbolId(0)],
            &mut ExpressionBudget::new(1_000_000, consumed),
        )
        .unwrap();
    assert_eq!(
        exact.program().definition(),
        jacobian.program().definition()
    );
    assert!(matches!(
        source.jacobian(
            &[SymbolId(0)],
            &mut ExpressionBudget::new(1_000_000, consumed - 1)
        ),
        Err(ExpressionError::WorkExhausted)
    ));
    let definition = jacobian.program().definition();
    let entries = definition.variables.len() + definition.nodes.len() + definition.outputs.len();
    assert!(matches!(
        source.jacobian(
            &[SymbolId(0)],
            &mut ExpressionBudget::new(entries - 1, 100_000_000)
        ),
        Err(ExpressionError::SizeExhausted)
    ));
    assert!(matches!(
        source.jacobian(&[SymbolId(2)], &mut budget()),
        Err(ExpressionError::UnboundVariable(SymbolId(2)))
    ));
    assert!(matches!(
        source.jacobian(&[SymbolId(0), SymbolId(0)], &mut budget()),
        Err(ExpressionError::DuplicateVariable(SymbolId(0)))
    ));
    let token = CancellationToken::new();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token.clone());
    token.cancel();
    assert!(matches!(
        source.jacobian(&[SymbolId(0)], &mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    let replay = ExpressionProgram::from_definition(source.definition().clone(), &mut budget())
        .unwrap()
        .jacobian(jacobian.variables(), &mut budget())
        .unwrap();
    assert_eq!(
        replay.program().definition(),
        jacobian.program().definition()
    );
    assert_eq!(
        source
            .jacobian(&[], &mut budget())
            .unwrap()
            .program()
            .definition(),
        source.definition()
    );
    let empty = compile(&[], &[SymbolId(0)])
        .jacobian(&[SymbolId(0)], &mut budget())
        .unwrap();
    assert_eq!(empty.partial_output(0, 0), None);
    assert!(
        empty
            .program()
            .eval_real(&[Real::zero()], &mut budget())
            .unwrap()
            .is_empty()
    );
}

#[cfg(feature = "interval-program")]
#[test]
fn interval_jacobian_has_independent_affine_entries_and_retained_guards() {
    use hyperinterval::{DyadicBudget, DyadicInterval};
    let d = |a: i64, b: i64| DyadicInterval::new(a.into(), b.into(), 0).unwrap();
    let source = compile(
        &[Expr::int(3) * x() + Expr::int(2) * Expr::anonymous_symbol(SymbolId(1))],
        &[SymbolId(0), SymbolId(1)],
    );
    let jacobian = source
        .jacobian(&[SymbolId(1), SymbolId(0)], &mut budget())
        .unwrap();
    assert_eq!(
        jacobian
            .program()
            .eval_dyadic(
                &[d(-2, 3), d(1, 4)],
                -32,
                &mut budget(),
                &mut DyadicBudget::new(16_384, 1_000_000_000)
            )
            .unwrap(),
        [d(-4, 17), d(2, 2), d(3, 3)]
    );
    let source = compile(
        &[x().powi(3) + Expr::int(2) * x() * Expr::anonymous_symbol(SymbolId(1))],
        &[SymbolId(0), SymbolId(1)],
    );
    let jacobian = source
        .jacobian(&[SymbolId(0), SymbolId(1)], &mut budget())
        .unwrap();
    let values = jacobian
        .program()
        .eval_dyadic(
            &[d(-2, 3), d(1, 4)],
            -32,
            &mut budget(),
            &mut DyadicBudget::new(16_384, 1_000_000_000),
        )
        .unwrap();
    // Independent extrema of 3*x^2+2*y and 2*x on the box.
    assert_eq!(values[1..], [d(2, 35), d(-4, 6)]);
    let source = compile(&[x().powi(0)], &[SymbolId(0)]);
    let jacobian = source.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
    assert!(matches!(
        jacobian.program().eval_dyadic(
            &[d(-1, 1)],
            -32,
            &mut budget(),
            &mut DyadicBudget::new(16_384, 1_000_000_000)
        ),
        Err(ExpressionError::DomainNotCertified { .. })
    ));
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_regenerates_partials_from_source_and_variables() {
    const KEY: &str = "HYPERSOLVE_JACOBIAN_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, variables): (ExpressionDefinition, Vec<SymbolId>) =
            serde_json::from_str(&encoded).unwrap();
        let source = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
        let jacobian = source.jacobian(&variables, &mut budget()).unwrap();
        let values = jacobian
            .program()
            .eval_real(&[Real::from(5)], &mut budget())
            .unwrap();
        assert_eq!(values[1], Real::from(10));
        println!(
            "JACOBIAN_REPLAY={}",
            serde_json::to_string(&values).unwrap()
        );
        return;
    }
    let source = compile(
        &[x().powi(2) + Expr::real(Real::from(2).sqrt().unwrap())],
        &[SymbolId(0)],
    );
    let jacobian = source.jacobian(&[SymbolId(0)], &mut budget()).unwrap();
    let encoded =
        serde_json::to_string(&(jacobian.source().definition(), jacobian.variables())).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_regenerates_partials_from_source_and_variables",
            "--nocapture",
        ])
        .env(KEY, &encoded)
        .output()
        .unwrap();
    assert!(
        child.status.success(),
        "{}",
        String::from_utf8_lossy(&child.stderr)
    );
    let text = String::from_utf8(child.stdout).unwrap();
    let replay = text
        .lines()
        .find_map(|line| line.strip_prefix("JACOBIAN_REPLAY="))
        .unwrap();
    let values = jacobian
        .program()
        .eval_real(&[Real::from(5)], &mut budget())
        .unwrap();
    assert_eq!(replay, serde_json::to_string(&values).unwrap());
    let (mut changed, variables): (ExpressionDefinition, Vec<SymbolId>) =
        serde_json::from_str(&encoded).unwrap();
    changed.outputs = vec![0];
    let changed = ExpressionProgram::from_definition(changed, &mut budget())
        .unwrap()
        .jacobian(&variables, &mut budget())
        .unwrap();
    assert_eq!(
        changed
            .program()
            .eval_real(&[Real::from(5)], &mut budget())
            .unwrap(),
        [Real::from(5), Real::one()]
    );
    assert_eq!(
        jacobian
            .program()
            .eval_real(&[Real::from(5)], &mut budget())
            .unwrap(),
        values
    );
}
