#![cfg(feature = "expression-program")]

use std::collections::HashMap;

use hyperreal::{Problem, Real};
use hypersolve::{
    Expr, ExprEvalError, SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
        ExpressionProgram, UnaryOperation,
    },
    work_budget::CancellationToken,
};

fn budget() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 1_000_000)
}

fn variable(id: u32) -> Expr {
    Expr::anonymous_symbol(SymbolId(id))
}

#[test]
fn all_operators_preserve_exact_values_and_extreme_exponents() {
    // Independent exact expectations, not just agreement between evaluators.
    let cases = vec![
        (Expr::int(7), 7),
        (variable(0), 4),
        (Expr::Add(Box::new(variable(0)), Box::new(Expr::int(3))), 7),
        (Expr::Sub(Box::new(variable(0)), Box::new(Expr::int(3))), 1),
        (Expr::Mul(Box::new(variable(0)), Box::new(Expr::int(3))), 12),
        (Expr::Div(Box::new(variable(0)), Box::new(Expr::int(2))), 2),
        (Expr::Neg(Box::new(variable(0))), -4),
        (variable(0).powi(3), 64),
        (variable(0).sqrt(), 2),
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
    let sources: Vec<_> = cases.iter().map(|(source, _)| source.clone()).collect();
    let expected: Vec<_> = cases.iter().map(|(_, value)| Real::from(*value)).collect();
    let program = ExpressionProgram::compile(&sources, &[SymbolId(0)], &mut budget()).unwrap();
    assert_eq!(
        program.eval_real(&[Real::from(4)], &mut budget()).unwrap(),
        expected
    );
    let bindings = HashMap::from([(SymbolId(0), Real::from(4))]);
    assert_eq!(
        sources
            .iter()
            .map(|expr| expr.eval_real(&bindings).unwrap())
            .collect::<Vec<_>>(),
        expected
    );
    // Rebuild independently from the complete definition, not a validity flag.
    let replay =
        ExpressionProgram::from_definition(program.definition().clone(), &mut budget()).unwrap();
    assert_eq!(replay.definition(), program.definition());
    assert_eq!(
        replay.eval_real(&[Real::from(4)], &mut budget()).unwrap(),
        expected
    );
}

#[test]
fn generated_polynomials_and_existing_symbolic_derivatives_are_consumable() {
    let x = SymbolId(9);
    for degree in 0..=12_i64 {
        let source = Expr::int(3) * Expr::anonymous_symbol(x).powi(degree) + Expr::int(7);
        let derivative = source.derivative(x);
        let program =
            ExpressionProgram::compile(&[source, derivative], &[x], &mut budget()).unwrap();
        for integer in [-3_i64, -1, 1, 2, 3] {
            let expected_source = 3_i64 * integer.pow(degree as u32) + 7;
            let expected_derivative = if degree == 0 {
                0
            } else {
                3 * degree * integer.pow((degree - 1) as u32)
            };
            assert_eq!(
                program
                    .eval_real(&[Real::from(integer)], &mut budget())
                    .unwrap(),
                [Real::from(expected_source), Real::from(expected_derivative)]
            );
        }
    }
    // Preserve the repaired boundary derivative and its original nonzero guard.
    let source = Expr::anonymous_symbol(x).powi(i64::MIN);
    let program =
        ExpressionProgram::compile(&[source.clone(), source.derivative(x)], &[x], &mut budget())
            .unwrap();
    assert_eq!(
        program.eval_real(&[Real::one()], &mut budget()).unwrap(),
        [Real::one(), Real::from(i64::MIN)]
    );
    assert!(matches!(
        program.eval_real(&[Real::zero()], &mut budget()),
        Err(ExpressionError::Scalar {
            source: Problem::NotANumber,
            ..
        })
    ));
}

#[test]
fn domain_failures_and_zero_factors_retain_original_guards() {
    let failures = vec![
        Expr::Div(Box::new(Expr::int(1)), Box::new(Expr::int(0))),
        Expr::int(-1).sqrt(),
        Expr::int(0).ln(),
        Expr::int(-1).log10(),
        Expr::int(2).asin(),
        Expr::int(2).acos(),
        Expr::int(0).acosh(),
        Expr::int(2).atanh(),
        Expr::int(0).powi(0),
        Expr::int(0).powi(-1),
    ];
    for failure in failures {
        for source in [
            failure.clone(),
            Expr::Mul(Box::new(Expr::int(0)), Box::new(failure.clone())),
            failure.powi(0),
        ] {
            let Err(ExprEvalError::Hyperreal(expected)) = source.eval_real(&HashMap::new()) else {
                panic!("invalid oracle fixture: {source:?}");
            };
            let program = ExpressionProgram::compile(&[source], &[], &mut budget()).unwrap();
            let error = program.eval_real(&[], &mut budget()).unwrap_err();
            let ExpressionError::Scalar { source, .. } = &error else {
                panic!("lost scalar error: {error}");
            };
            assert_eq!(source, &expected);
            assert!(std::error::Error::source(&error).is_some());
        }
    }
}

#[test]
fn binding_order_ownership_and_structural_identity_remain_explicit() {
    let mut sources = [Expr::Sub(Box::new(variable(7)), Box::new(variable(2)))];
    let program =
        ExpressionProgram::compile(&sources, &[SymbolId(2), SymbolId(7)], &mut budget()).unwrap();
    sources[0] = Expr::int(99);
    assert_eq!(
        program
            .eval_real(&[Real::from(3), Real::from(8)], &mut budget())
            .unwrap(),
        [Real::from(5)]
    );
    assert!(std::ptr::eq(
        program.definition(),
        program.clone().definition()
    ));
    let replay =
        ExpressionProgram::from_definition(program.definition().clone(), &mut budget()).unwrap();
    assert!(!std::ptr::eq(program.definition(), replay.definition()));
    assert_eq!(program.definition(), replay.definition());
    let mut swapped = program.definition().clone();
    swapped.variables.swap(0, 1);
    let swapped = ExpressionProgram::from_definition(swapped, &mut budget()).unwrap();
    assert_ne!(program.definition(), swapped.definition());
    assert_eq!(
        swapped
            .eval_real(&[Real::from(3), Real::from(8)], &mut budget())
            .unwrap(),
        [Real::from(-5)]
    );
    for values in [vec![], vec![Real::one()], vec![Real::one(); 3]] {
        assert!(matches!(
            program.eval_real(&values, &mut budget()),
            Err(ExpressionError::BindingCount { expected: 2, .. })
        ));
    }
}

#[test]
fn replay_rejects_bad_references_and_bindings_even_without_outputs() {
    let definition = |nodes, variables, outputs| ExpressionDefinition {
        nodes,
        variables,
        outputs,
    };
    for input in [0, 1, usize::MAX] {
        let result = ExpressionProgram::from_definition(
            definition(
                vec![ExpressionNode::Unary {
                    operation: UnaryOperation::Neg,
                    input,
                }],
                vec![],
                vec![],
            ),
            &mut budget(),
        );
        assert!(
            matches!(result, Err(ExpressionError::InvalidReference { node: 0, input: found }) if found == input)
        );
    }
    assert!(matches!(
        ExpressionProgram::from_definition(
            definition(vec![], vec![SymbolId(0); 2], vec![]),
            &mut budget()
        ),
        Err(ExpressionError::DuplicateVariable(SymbolId(0)))
    ));
    assert!(matches!(
        ExpressionProgram::compile(&[variable(1)], &[], &mut budget()),
        Err(ExpressionError::UnboundVariable(SymbolId(1)))
    ));
    assert!(matches!(
        ExpressionProgram::from_definition(definition(vec![], vec![], vec![0]), &mut budget()),
        Err(ExpressionError::InvalidOutput { output: 0, node: 0 })
    ));
    // The right reference must be checked as well as the left.
    let nodes = vec![
        ExpressionNode::Constant(Real::one()),
        ExpressionNode::Binary {
            operation: BinaryOperation::Add,
            left: 0,
            right: 2,
        },
    ];
    assert!(matches!(
        ExpressionProgram::from_definition(definition(nodes, vec![], vec![0]), &mut budget()),
        Err(ExpressionError::InvalidReference { node: 1, input: 2 })
    ));
}

#[test]
fn demand_order_reuse_and_unused_nodes_do_not_change_domain() {
    let definition = ExpressionDefinition {
        variables: vec![],
        nodes: vec![
            ExpressionNode::Constant(Real::from(-1)),
            ExpressionNode::Unary {
                operation: UnaryOperation::Sqrt,
                input: 0,
            },
            ExpressionNode::Constant(Real::zero()),
            ExpressionNode::Binary {
                operation: BinaryOperation::Div,
                left: 0,
                right: 2,
            },
            // Node-index order would fail at sqrt; demand order must fail at Div.
            ExpressionNode::Binary {
                operation: BinaryOperation::Add,
                left: 3,
                right: 1,
            },
            ExpressionNode::Binary {
                operation: BinaryOperation::Add,
                left: 0,
                right: 0,
            },
        ],
        outputs: vec![5, 5, 2],
    };
    let program = ExpressionProgram::from_definition(definition.clone(), &mut budget()).unwrap();
    assert_eq!(
        program.eval_real(&[], &mut budget()).unwrap(),
        [Real::from(-2), Real::from(-2), Real::zero()]
    );
    let mut invalid = definition.clone();
    invalid.outputs = vec![4, 1];
    let program = ExpressionProgram::from_definition(invalid, &mut budget()).unwrap();
    assert!(matches!(
        program.eval_real(&[], &mut budget()),
        Err(ExpressionError::Scalar { node: 3, .. })
    ));
    let mut invalid = definition;
    invalid.outputs = vec![1, 4];
    let program = ExpressionProgram::from_definition(invalid, &mut budget()).unwrap();
    assert!(matches!(
        program.eval_real(&[], &mut budget()),
        Err(ExpressionError::Scalar { node: 1, .. })
    ));
}

#[test]
fn structural_limits_cancellation_and_retry_publish_no_partial_value() {
    let source = [Expr::Add(Box::new(variable(0)), Box::new(Expr::int(2)))];
    // Three nodes, one binding, one output; 3N + 2V + 2O compile units.
    let mut exact = ExpressionBudget::new(5, 13);
    let program = ExpressionProgram::compile(&source, &[SymbolId(0)], &mut exact).unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        ExpressionProgram::compile(&source, &[SymbolId(0)], &mut ExpressionBudget::new(4, 100)),
        Err(ExpressionError::SizeExhausted)
    ));
    for work in 0..13 {
        assert!(matches!(
            ExpressionProgram::compile(
                &source,
                &[SymbolId(0)],
                &mut ExpressionBudget::new(5, work)
            ),
            Err(ExpressionError::WorkExhausted)
        ));
    }
    for work in 0..20 {
        assert!(matches!(
            program.eval_real(&[Real::from(3)], &mut ExpressionBudget::new(5, work)),
            Err(ExpressionError::WorkExhausted)
        ));
    }
    let token = CancellationToken::new();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token.clone());
    token.cancel();
    let before = cancelled.remaining_work();
    assert!(matches!(
        program.eval_real(&[Real::from(3)], &mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    assert!(matches!(
        ExpressionProgram::compile(&source, &[SymbolId(0)], &mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    assert!(matches!(
        ExpressionProgram::from_definition(program.definition().clone(), &mut cancelled),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(cancelled.remaining_work(), before);
    assert_eq!(
        program
            .eval_real(&[Real::from(3)], &mut ExpressionBudget::new(5, 20))
            .unwrap(),
        [Real::from(5)]
    );
}

#[test]
fn empty_outputs_are_valid_and_still_validate_argument_count() {
    let program = ExpressionProgram::compile(&[], &[SymbolId(0)], &mut budget()).unwrap();
    assert!(
        program
            .eval_real(&[Real::one()], &mut budget())
            .unwrap()
            .is_empty()
    );
    assert!(matches!(
        program.eval_real(&[], &mut budget()),
        Err(ExpressionError::BindingCount { .. })
    ));
    let empty = ExpressionProgram::compile(&[], &[], &mut ExpressionBudget::new(0, 0)).unwrap();
    assert!(
        empty
            .eval_real(&[], &mut ExpressionBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
}

#[test]
fn deep_source_compiles_evaluates_and_drops_on_a_small_stack() {
    std::thread::Builder::new()
        .stack_size(128 * 1024)
        .spawn(|| {
            let mut source = variable(0);
            for _ in 0..20_000 {
                source = Expr::Neg(Box::new(source));
            }
            let rejected = ExpressionProgram::compile(
                std::slice::from_ref(&source),
                &[SymbolId(0)],
                &mut ExpressionBudget::new(20, 100),
            );
            let result = ExpressionProgram::compile(
                std::slice::from_ref(&source),
                &[SymbolId(0)],
                &mut budget(),
            );
            // Caller-owned Expr has recursive drop. Dismantle it iteratively before
            // assertions, including if import fails; graph cleanup is already flat.
            while let Expr::Neg(inner) = source {
                source = *inner;
            }
            assert!(matches!(rejected, Err(ExpressionError::SizeExhausted)));
            let program = result.unwrap();
            assert_eq!(program.definition().nodes.len(), 20_001);
            assert_eq!(
                program.eval_real(&[Real::from(17)], &mut budget()).unwrap(),
                [Real::from(17)]
            );
        })
        .unwrap()
        .join()
        .unwrap();
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_retains_symbolic_constants_and_revalidates_edges() {
    use std::process::Command;
    let source = Expr::Add(
        Box::new(variable(3)),
        Box::new(Expr::real(Real::from(2).sqrt().unwrap())),
    );
    let program = ExpressionProgram::compile(&[source], &[SymbolId(3)], &mut budget()).unwrap();
    let path = std::env::temp_dir().join(format!(
        "hypersolve-program-replay-{}.json",
        std::process::id()
    ));
    struct Remove(std::path::PathBuf);
    impl Drop for Remove {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let _remove = Remove(path.clone());
    std::fs::write(&path, serde_json::to_vec(program.definition()).unwrap()).unwrap();
    let run = || {
        Command::new(replay_binary::path().unwrap())
            .args(["--exact", "replay_child", "--nocapture"])
            .env("HYPERSOLVE_PROGRAM_REPLAY", &path)
            .output()
            .unwrap()
    };
    let child = run();
    assert!(
        child.status.success(),
        "{}",
        String::from_utf8_lossy(&child.stderr)
    );
    let stdout = String::from_utf8(child.stdout).unwrap();
    let encoded = stdout
        .lines()
        .find_map(|line| line.strip_prefix("REPLAY="))
        .unwrap();
    let replayed: Vec<Real> = serde_json::from_str(encoded).unwrap();
    assert_eq!(replayed, [Real::from(5) + Real::from(2).sqrt().unwrap()]);
    #[cfg(feature = "interval-program")]
    {
        let encoded = stdout
            .lines()
            .find_map(|line| line.strip_prefix("INTERVAL_REPLAY="))
            .unwrap();
        let replayed: Vec<hyperinterval::DyadicInterval> = serde_json::from_str(encoded).unwrap();
        // Independent integer-square-root witness for floor(sqrt(2)*2^32).
        let lower = num::BigInt::from((5_u128 << 32) + (2_u128 << 64).isqrt());
        let minimal = hyperinterval::DyadicInterval::new(lower.clone(), lower + 1, -32).unwrap();
        let mut arithmetic = hyperinterval::DyadicBudget::new(16384, 1_000_000_000_000);
        assert_eq!(replayed.len(), 1);
        assert!(minimal.subset_of(&replayed[0], &mut arithmetic).unwrap());
        // Hyperreal promises certified grid bounds, not a one-cell enclosure.
        // Check replay equality separately from the independent root witness.
        let live = program
            .eval_dyadic(
                &[hyperinterval::DyadicInterval::point(5.into(), 0).unwrap()],
                -32,
                &mut budget(),
                &mut arithmetic,
            )
            .unwrap();
        assert_eq!(replayed, live);
    }
    let mut bad = program.definition().clone();
    bad.nodes[2] = ExpressionNode::Unary {
        operation: UnaryOperation::Neg,
        input: 2,
    };
    std::fs::write(&path, serde_json::to_vec(&bad).unwrap()).unwrap();
    let child = run();
    assert!(child.status.success());
    assert!(
        String::from_utf8(child.stdout)
            .unwrap()
            .contains("INVALID_REFERENCE")
    );
}

#[cfg(feature = "serde")]
#[test]
fn replay_child() {
    let Some(path) = std::env::var_os("HYPERSOLVE_PROGRAM_REPLAY") else {
        return;
    };
    let definition: ExpressionDefinition =
        serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
    match ExpressionProgram::from_definition(definition, &mut budget()) {
        Ok(program) => {
            println!(
                "REPLAY={}",
                serde_json::to_string(&program.eval_real(&[Real::from(5)], &mut budget()).unwrap())
                    .unwrap()
            );
            #[cfg(feature = "interval-program")]
            {
                let values = [hyperinterval::DyadicInterval::point(5.into(), 0).unwrap()];
                let image = program
                    .eval_dyadic(
                        &values,
                        -32,
                        &mut budget(),
                        &mut hyperinterval::DyadicBudget::new(16384, 1_000_000_000_000),
                    )
                    .unwrap();
                println!("INTERVAL_REPLAY={}", serde_json::to_string(&image).unwrap());
            }
        }
        Err(ExpressionError::InvalidReference { .. }) => println!("INVALID_REFERENCE"),
        Err(error) => panic!("unexpected replay failure: {error}"),
    }
}

#[test]
fn symbol_metadata_is_retained_without_aliasing_bindings() {
    let source = Expr::Symbol(SymbolRef::new(SymbolId(5), Some("time".into())));
    let program = ExpressionProgram::compile(&[source], &[SymbolId(5)], &mut budget()).unwrap();
    assert!(
        matches!(&program.definition().nodes[0], ExpressionNode::Variable(reference) if reference.name.as_deref() == Some("time"))
    );
    assert_eq!(
        program.eval_real(&[Real::from(7)], &mut budget()).unwrap(),
        [Real::from(7)]
    );
}

#[test]
fn shared_generated_graphs_match_independent_integer_expectations() {
    let mut seed = 0x83a7_b209_11de_45ff_u64;
    for case in 0..400 {
        let point = i128::from(case % 5) - 2;
        let mut nodes = vec![
            ExpressionNode::Constant(Real::from(-3)),
            ExpressionNode::Constant(Real::from(2)),
            ExpressionNode::Variable(SymbolRef::new(SymbolId(7), None)),
        ];
        let mut expected = vec![-3_i128, 2, point];
        for index in 3..32 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let left = seed as usize % index;
            let right = (seed >> 32) as usize % index;
            let (node, value) = match seed % 4 {
                0 => (
                    ExpressionNode::Binary {
                        operation: BinaryOperation::Add,
                        left,
                        right,
                    },
                    expected[left] + expected[right],
                ),
                1 => (
                    ExpressionNode::Binary {
                        operation: BinaryOperation::Sub,
                        left,
                        right,
                    },
                    expected[left] - expected[right],
                ),
                // A small constant multiplier keeps this independent oracle
                // within i128; the graph still includes arbitrary shared edges.
                2 => (
                    ExpressionNode::Binary {
                        operation: BinaryOperation::Mul,
                        left,
                        right: right % 2,
                    },
                    expected[left] * expected[right % 2],
                ),
                _ => (
                    ExpressionNode::Unary {
                        operation: UnaryOperation::Neg,
                        input: left,
                    },
                    -expected[left],
                ),
            };
            nodes.push(node);
            expected.push(value);
        }
        let mut outputs = vec![31, 2, 15, 31, 0, 30, 15];
        if case % 2 == 0 {
            outputs.reverse();
        }
        let expected: Vec<_> = outputs
            .iter()
            .map(|&index| Real::from(expected[index]))
            .collect();
        let definition = ExpressionDefinition {
            variables: vec![SymbolId(7)],
            nodes,
            outputs,
        };
        let program = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
        assert_eq!(
            program
                .eval_real(&[Real::from(point)], &mut budget())
                .unwrap(),
            expected,
            "graph {case}"
        );
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn tree_import_preserves_warm_scalar_dispatch_on_the_polynomial_fixture() {
    // Small scalar constants share caches across tests. Isolate the measured
    // fixture so another test cannot alter its count while it is recording.
    if std::env::var_os("HYPERSOLVE_PROGRAM_TRACE_CHILD").is_none() {
        let child = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "tree_import_preserves_warm_scalar_dispatch_on_the_polynomial_fixture",
                "--nocapture",
            ])
            .env("HYPERSOLVE_PROGRAM_TRACE_CHILD", "1")
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        return;
    }
    use hyperreal::dispatch_trace;
    let source = Expr::int(3) * variable(0).powi(3) - Expr::int(7);
    dispatch_trace::reset();
    let program = dispatch_trace::with_recording(|| {
        ExpressionProgram::compile(std::slice::from_ref(&source), &[SymbolId(0)], &mut budget())
            .unwrap()
    });
    let import_trace = dispatch_trace::take_trace();
    assert!(import_trace.dispatch.is_empty());
    assert_eq!(import_trace.rational, Default::default());
    let bindings = HashMap::from([(SymbolId(0), Real::from(2))]);
    let values = [Real::from(2)];
    // Match cache state; comparing the first scalar observation against a
    // later observation of retained input constants is not a matched trace.
    for _ in 0..8 {
        source.eval_real(&bindings).unwrap();
        program.eval_real(&values, &mut budget()).unwrap();
    }
    dispatch_trace::reset();
    let expected = dispatch_trace::with_recording(|| source.eval_real(&bindings).unwrap());
    let expected_trace = dispatch_trace::take_trace();
    dispatch_trace::reset();
    let actual =
        dispatch_trace::with_recording(|| program.eval_real(&values, &mut budget()).unwrap());
    let actual_trace = dispatch_trace::take_trace();
    assert_eq!(actual, [Real::from(17)]);
    assert_eq!(actual, [expected]);
    assert_eq!(actual_trace, expected_trace);
    let counts = actual_trace.correlation_summary();
    println!("polynomial trace: {counts:?}");
    assert_eq!(counts.dispatch_events, 11);
    assert_eq!(counts.rational_temporaries, 0);
    assert_eq!(counts.rational_gcds, 0);
    assert_eq!(counts.sign_or_zero_query_events, 0);
}

#[cfg(any(feature = "serde", feature = "dispatch-trace"))]
#[path = "support/replay_binary.rs"]
mod replay_binary;
