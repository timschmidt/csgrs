#![cfg(feature = "coordinate-program")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
#[test]
fn shear_translation_has_independent_polynomial_and_jacobian_oracles() {
    let x = Expr::anonymous_symbol(SymbolId(0));
    let y = Expr::anonymous_symbol(SymbolId(1));
    let t = Expr::anonymous_symbol(SymbolId(2));
    let source = ExpressionProgram::compile(
        &[x.clone().powi(2) + y.clone() + t, y.powi(2) - x],
        &[SymbolId(1), SymbolId(2), SymbolId(0)],
        &mut budget(),
    )
    .unwrap();
    let chart = source
        .change_coordinates(
            CoordinateChange {
                variables: vec![SymbolId(0), SymbolId(1)],
                to_source: vec![vec![1.into(), 2.into()], vec![0.into(), 1.into()]],
                from_source: vec![vec![1.into(), (-2).into()], vec![0.into(), 1.into()]],
                origin: vec![3.into(), (-1).into()],
            },
            &mut budget(),
        )
        .unwrap();
    let jac = chart
        .program()
        .jacobian(&[SymbolId(0), SymbolId(1)], &mut budget())
        .unwrap();
    for a in -2_i64..=2 {
        for b in -2_i64..=2 {
            for t in -1_i64..=1 {
                let x = 3 + a + 2 * b;
                let y = b - 1;
                let expected = [
                    x * x + y + t - 2 * (y * y - x),
                    y * y - x,
                    2 * x + 2,
                    -1,
                    4 * x + 5 - 4 * y,
                    2 * y - 2,
                ]
                .map(Real::from);
                assert_eq!(
                    jac.program()
                        .eval_real(&[b.into(), t.into(), a.into()], &mut budget())
                        .unwrap(),
                    expected
                );
            }
        }
    }
}

fn identity_chart(n: usize) -> CoordinateChange {
    CoordinateChange {
        variables: (0..n)
            .map(|i| SymbolId(u32::try_from(i).unwrap()))
            .collect(),
        to_source: (0..n)
            .map(|i| (0..n).map(|j| Real::from(u8::from(i == j))).collect())
            .collect(),
        from_source: (0..n)
            .map(|i| (0..n).map(|j| Real::from(u8::from(i == j))).collect())
            .collect(),
        origin: vec![Real::zero(); n],
    }
}
fn linear_source(n: usize) -> ExpressionProgram {
    let variables: Vec<_> = (0..n)
        .map(|i| SymbolId(u32::try_from(i).unwrap()))
        .collect();
    ExpressionProgram::compile(
        &variables
            .iter()
            .map(|&v| Expr::anonymous_symbol(v))
            .collect::<Vec<_>>(),
        &variables,
        &mut budget(),
    )
    .unwrap()
}
#[test]
fn inverse_admission_distinguishes_wrong_unknown_and_algebraic_certified() {
    use hyperlimit::{Certainty, PredicateOutcome, Sign};
    use hypersolve::expression_program::CoordinateError;
    let source = linear_source(1);
    let mut chart = identity_chart(1);
    chart.to_source[0][0] = Real::from(2).sqrt().unwrap();
    chart.from_source[0][0] = (chart.to_source[0][0].clone() / Real::from(2)).unwrap();
    let admitted = source.change_coordinates(chart, &mut budget()).unwrap();
    assert!(matches!(
        admitted.inverse_evidence(),
        [PredicateOutcome::Decided {
            value: Sign::Zero,
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        }]
    ));
    assert_eq!(
        admitted
            .program()
            .eval_real(&[3.into()], &mut budget())
            .unwrap(),
        vec![Real::from(3)]
    );
    let mut invalid = identity_chart(1);
    invalid.from_source[0][0] = Real::from(2);
    assert!(matches!(
        source.change_coordinates(invalid, &mut budget()),
        Err(CoordinateError::NotInverse {
            row: 0,
            column: 0,
            ..
        })
    ));
    let mut singular = identity_chart(1);
    singular.to_source[0][0] = Real::zero();
    assert!(matches!(
        source.change_coordinates(singular, &mut budget()),
        Err(CoordinateError::NotInverse { .. })
    ));
    // Retained unresolved angle-addition identity from Hyperlimit's terminal-policy corpus.
    let unresolved = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    let mut unknown = identity_chart(1);
    unknown.from_source[0][0] = Real::one() + unresolved;
    assert!(matches!(
        source.change_coordinates(unknown, &mut budget()),
        Err(CoordinateError::InverseUndecided {
            outcome: PredicateOutcome::Unknown { .. },
            ..
        })
    ));
}
#[test]
fn invalid_shapes_bindings_and_unit_chart_are_checked() {
    use hypersolve::expression_program::{CoordinateError, ExpressionError};
    let source = linear_source(2);
    let mut invalid = identity_chart(2);
    invalid.to_source[0].pop();
    assert!(matches!(
        source.change_coordinates(invalid, &mut budget()),
        Err(CoordinateError::Shape {
            field: "to_source",
            row: Some(0),
            ..
        })
    ));
    let mut invalid = identity_chart(2);
    invalid.variables[1] = SymbolId(0);
    assert!(matches!(
        source.change_coordinates(invalid, &mut budget()),
        Err(CoordinateError::Expression(
            ExpressionError::DuplicateVariable(_)
        ))
    ));
    let mut invalid = identity_chart(2);
    invalid.variables[1] = SymbolId(9);
    assert!(matches!(
        source.change_coordinates(invalid, &mut budget()),
        Err(CoordinateError::Expression(
            ExpressionError::UnboundVariable(_)
        ))
    ));
    let unit = linear_source(0)
        .change_coordinates(identity_chart(0), &mut budget())
        .unwrap();
    assert!(
        unit.program()
            .eval_real(&[], &mut budget())
            .unwrap()
            .is_empty()
    );
    assert!(unit.inverse_evidence().is_empty());
    assert!(matches!(
        linear_source(0).change_coordinates(identity_chart(1), &mut budget()),
        Err(CoordinateError::Shape {
            field: "outputs",
            ..
        })
    ));
}
#[test]
fn guards_sharing_and_finite_composition_survive_coordinate_changes() {
    use hypersolve::{
        SymbolRef,
        expression_program::{BinaryOperation, ExpressionDefinition, ExpressionNode},
    };
    let x = Expr::anonymous_symbol(SymbolId(0));
    let y = Expr::anonymous_symbol(SymbolId(1));
    let pole = ExpressionProgram::compile(
        &[Expr::int(1) / x, y],
        &[SymbolId(0), SymbolId(1)],
        &mut budget(),
    )
    .unwrap();
    let chart = pole
        .change_coordinates(identity_chart(2), &mut budget())
        .unwrap();
    assert!(
        chart
            .program()
            .eval_real(&[0.into(), 1.into()], &mut budget())
            .is_err()
    );
    #[cfg(feature = "interval-program")]
    {
        use hyperinterval::{DyadicBudget, DyadicInterval};
        let values = [
            DyadicInterval::new((-1).into(), 1.into(), 0).unwrap(),
            DyadicInterval::point(1.into(), 0).unwrap(),
        ];
        assert!(
            chart
                .program()
                .eval_dyadic(
                    &values,
                    -32,
                    &mut budget(),
                    &mut DyadicBudget::new(16384, 1_000_000_000)
                )
                .is_err()
        );
    }
    let shared = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![SymbolId(0)],
            nodes: vec![
                ExpressionNode::Variable(SymbolRef::new(SymbolId(0), Some("first".into()))),
                ExpressionNode::Variable(SymbolRef::new(SymbolId(0), Some("second".into()))),
                ExpressionNode::Binary {
                    operation: BinaryOperation::Add,
                    left: 0,
                    right: 1,
                },
            ],
            outputs: vec![2],
        },
        &mut budget(),
    )
    .unwrap();
    let result = shared
        .change_coordinates(identity_chart(1), &mut budget())
        .unwrap();
    assert_eq!(result.source().definition(), shared.definition());
    assert_eq!(result.transformed_node(0), result.transformed_node(1));
    assert_eq!(result.transformed_node(3), None);
    let mut current = linear_source(2);
    for _ in 0..8 {
        for k in [2, -2] {
            let mut change = identity_chart(2);
            change.to_source[0][1] = k.into();
            change.from_source[0][1] = (-k).into();
            current = current
                .change_coordinates(change, &mut budget())
                .unwrap()
                .program()
                .clone();
        }
    }
    assert_eq!(
        current
            .eval_real(&[3.into(), 4.into()], &mut budget())
            .unwrap(),
        vec![Real::from(3), Real::from(4)]
    );
    assert!(current.definition().nodes.len() < 600);
}
#[test]
fn exact_work_cutoffs_cancellation_and_retry_preserve_original() {
    use hypersolve::{
        expression_program::{CoordinateError, ExpressionError},
        work_budget::CancellationToken,
    };
    let source = linear_source(2);
    let before = source.definition().clone();
    let mut full = budget();
    source
        .change_coordinates(identity_chart(2), &mut full)
        .unwrap();
    let used = budget().remaining_work() - full.remaining_work();
    let mut exact = ExpressionBudget::new(1_000_000, used);
    source
        .change_coordinates(identity_chart(2), &mut exact)
        .unwrap();
    assert_eq!(exact.remaining_work(), 0);
    for work in [0, 1, used - 1] {
        assert!(matches!(
            source.change_coordinates(
                identity_chart(2),
                &mut ExpressionBudget::new(1_000_000, work)
            ),
            Err(CoordinateError::Expression(ExpressionError::WorkExhausted))
        ));
    }
    assert!(matches!(
        source.change_coordinates(identity_chart(2), &mut ExpressionBudget::new(1, usize::MAX)),
        Err(CoordinateError::Expression(ExpressionError::SizeExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        source.change_coordinates(identity_chart(2), &mut cancelled),
        Err(CoordinateError::Expression(ExpressionError::Cancelled))
    ));
    assert_eq!(source.definition(), &before);
    source
        .change_coordinates(identity_chart(2), &mut budget())
        .unwrap();
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_inverse_and_source_association() {
    const KEY: &str = "HYPERSOLVE_COORDINATE_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, change): (
            hypersolve::expression_program::ExpressionDefinition,
            CoordinateChange,
        ) = serde_json::from_str(&data).unwrap();
        let mut bad = definition.clone();
        bad.outputs[0] = usize::MAX;
        assert!(ExpressionProgram::from_definition(bad, &mut budget()).is_err());
        let source = ExpressionProgram::from_definition(definition, &mut budget()).unwrap();
        let mut bad = change.clone();
        bad.from_source[0][0] = Real::from(7);
        assert!(source.change_coordinates(bad, &mut budget()).is_err());
        let mut work = budget();
        let chart = source.change_coordinates(change, &mut work).unwrap();
        let value = chart
            .program()
            .eval_real(&[3.into(), 4.into()], &mut work)
            .unwrap();
        assert_eq!(value, vec![Real::from(3), Real::from(4)]);
        println!(
            "CHART {} {:?} {}",
            serde_json::to_string(chart.program().definition()).unwrap(),
            chart.inverse_evidence(),
            work.remaining_work()
        );
        return;
    }
    let source = linear_source(2);
    let mut change = identity_chart(2);
    change.to_source[0][1] = 2.into();
    change.from_source[0][1] = (-2).into();
    let data = serde_json::to_string(&(source.definition(), change)).unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_rebuilds_inverse_and_source_association",
                "--nocapture",
            ])
            .env(KEY, &data)
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{} {}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        let text = String::from_utf8(child.stdout).unwrap();
        let line = text
            .lines()
            .find(|l| l.starts_with("CHART "))
            .unwrap()
            .to_owned();
        if let Some(expected) = previous {
            assert_eq!(line, expected);
        }
        previous = Some(line);
    }
}

#[test]
fn dense_four_dimensional_rational_chart_preserves_binding_order() {
    let ids = [SymbolId(0), SymbolId(1), SymbolId(2), SymbolId(3)];
    let outputs: Vec<_> = ids
        .iter()
        .enumerate()
        .map(|(i, &id)| Expr::int(i64::try_from(i + 1).unwrap()) * Expr::anonymous_symbol(id))
        .collect();
    let source =
        ExpressionProgram::compile(&outputs, &[ids[3], ids[1], ids[0], ids[2]], &mut budget())
            .unwrap();
    let hadamard = [[1, 1, 1, 1], [1, -1, 1, -1], [1, 1, -1, -1], [1, -1, -1, 1]];
    let change = CoordinateChange {
        variables: ids.to_vec(),
        to_source: hadamard
            .iter()
            .map(|r| r.iter().map(|&x| Real::from(x)).collect())
            .collect(),
        from_source: hadamard
            .iter()
            .map(|r| {
                r.iter()
                    .map(|&x| (Real::from(x) / Real::from(4)).unwrap())
                    .collect()
            })
            .collect(),
        origin: vec![Real::zero(); 4],
    };
    let chart = source.change_coordinates(change, &mut budget()).unwrap();
    assert_eq!(chart.inverse_evidence().len(), 16);
    // H*(1,2,3,4)=(10,-2,-4,0), diagonal field=(10,-4,-12,0).
    let expected = [-3, 1, 9, 13].map(|x| (Real::from(x) / Real::from(2)).unwrap());
    assert_eq!(
        chart
            .program()
            .eval_real(&[4.into(), 2.into(), 1.into(), 3.into()], &mut budget())
            .unwrap(),
        expected
    );
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
