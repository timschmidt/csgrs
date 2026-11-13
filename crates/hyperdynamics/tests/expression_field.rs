#![cfg(feature = "field")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    FieldError,
};
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalMatrix, IntervalVector};
#[cfg(feature = "serde")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode, ExpressionProgram,
        UnaryOperation,
    },
    work_budget::CancellationToken,
};
const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
const V: SymbolId = SymbolId(30);
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn variable(id: SymbolId) -> Expr {
    Expr::anonymous_symbol(id)
}
fn definition(outputs: &[Expr], variables: &[SymbolId], state: &[SymbolId]) -> FieldDefinition {
    let source = ExpressionProgram::compile(
        outputs,
        variables,
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    FieldDefinition {
        source: source.definition().clone(),
        time: T,
        state: state.to_vec(),
    }
}
fn affine(a: i64, c: i64, e: i64, k: i64) -> Expr {
    Expr::int(a) * variable(T)
        + Expr::int(c) * variable(X)
        + Expr::int(e) * variable(V)
        + Expr::int(k)
}
fn extrema(terms: &[(i64, i64, i64)], k: i64) -> DyadicInterval {
    let mut lo = k;
    let mut hi = k;
    for &(coefficient, a, b) in terms {
        lo += (coefficient * a).min(coefficient * b);
        hi += (coefficient * a).max(coefficient * b);
    }
    d(lo, hi)
}

#[test]
fn reordered_time_and_state_bindings_have_independent_affine_images() {
    for a in -2..=2 {
        for c in -2..=2 {
            for e in -2..=2 {
                let field = ExpressionField::<2>::from_definition(
                    definition(
                        &[affine(a, c, e, 3), affine(e, -a, c, -1)],
                        &[V, T, X],
                        &[X, V],
                    ),
                    &mut b(),
                )
                .unwrap();
                let state = BoxN::new([d(-2, 3), d(1, 4)]).unwrap();
                let mut budget = b();
                let image = field.image(&d(-1, 2), &state, &mut budget).unwrap();
                assert_eq!(
                    image.lanes(),
                    Some(&[
                        extrema(&[(a, -1, 2), (c, -2, 3), (e, 1, 4)], 3),
                        extrema(&[(e, -1, 2), (-a, -2, 3), (c, 1, 4)], -1)
                    ])
                );
                let jacobian = field.jacobian(&d(-1, 2), &state, &mut budget).unwrap();
                assert_eq!(
                    jacobian.rows(),
                    Some(&[[d(c, c), d(e, e)], [d(-a, -a), d(c, c)]])
                );
                let vector = IntervalVector::new([d(-1, 1), d(2, 3)]).unwrap();
                let product = jacobian
                    .mul_vector(&vector, &mut DyadicBudget::new(16384, 1_000_000_000))
                    .unwrap();
                assert_eq!(
                    product.lanes(),
                    Some(&[
                        extrema(&[(c, -1, 1), (e, 2, 3)], 0),
                        extrema(&[(-a, -1, 1), (c, 2, 3)], 0)
                    ])
                );
            }
        }
    }
}

#[test]
fn nonlinear_fields_keep_the_source_domain_and_distinguish_derivative_failure() {
    let field = ExpressionField::<1>::from_definition(
        definition(&[variable(X).powi(3) + variable(T)], &[X, T], &[X]),
        &mut b(),
    )
    .unwrap();
    let state = BoxN::new([d(1, 2)]).unwrap();
    assert_eq!(
        field.image(&d(0, 1), &state, &mut b()).unwrap().lanes(),
        Some(&[d(1, 9)])
    );
    assert_eq!(
        field.jacobian(&d(0, 1), &state, &mut b()).unwrap().rows(),
        Some(&[[d(3, 12)]])
    );
    let root = ExpressionField::<1>::from_definition(
        definition(&[variable(X).sqrt()], &[T, X], &[X]),
        &mut b(),
    )
    .unwrap();
    assert_eq!(
        root.image(&d(0, 0), &BoxN::new([d(0, 0)]).unwrap(), &mut b())
            .unwrap()
            .lanes(),
        Some(&[d(0, 0)])
    );
    assert!(matches!(
        root.jacobian(&d(0, 0), &BoxN::new([d(0, 0)]).unwrap(), &mut b()),
        Err(FieldError::Expression(ExpressionError::Enclosure {
            source: IntervalError::ContainsZero,
            ..
        }))
    ));
    let guarded = ExpressionField::<1>::from_definition(
        definition(&[variable(X).powi(0)], &[T, X], &[X]),
        &mut b(),
    )
    .unwrap();
    assert!(matches!(
        guarded.jacobian(&d(0, 1), &BoxN::new([d(-1, 1)]).unwrap(), &mut b()),
        Err(FieldError::Expression(
            ExpressionError::DomainNotCertified { .. }
        ))
    ));
    let constant = ExpressionField::<1>::from_definition(
        definition(&[Expr::int(0).sqrt()], &[T, X], &[X]),
        &mut b(),
    )
    .unwrap();
    assert_eq!(
        constant
            .jacobian(&d(0, 0), &BoxN::new([d(0, 0)]).unwrap(), &mut b())
            .unwrap()
            .rows(),
        Some(&[[d(0, 0)]])
    );
}

#[test]
fn canonical_empty_and_zero_dimensional_unit_products_remain_distinct() {
    let unit = ExpressionField::<0>::from_definition(definition(&[], &[T], &[]), &mut b()).unwrap();
    let state = BoxN::new([]).unwrap();
    assert_eq!(
        unit.image(&d(0, 1), &state, &mut b()).unwrap(),
        IntervalVector::new([]).unwrap()
    );
    assert_eq!(
        unit.jacobian(&d(0, 1), &state, &mut b()).unwrap(),
        IntervalMatrix::new([]).unwrap()
    );
    for (time, state) in [
        (DyadicInterval::empty(), state),
        (d(0, 1), BoxN::empty().unwrap()),
    ] {
        let mut budget = EnclosureBudget::new(EnclosurePolicy {
            scalar_bits: 0,
            scalar_work: 0,
            ..EnclosurePolicy::default()
        });
        assert!(unit.image(&time, &state, &mut budget).unwrap().is_empty());
        assert!(
            unit.jacobian(&time, &state, &mut budget)
                .unwrap()
                .is_empty()
        );
        assert_eq!(budget.remaining_scalar_work(), 0);
    }
    let invalid = FieldDefinition {
        source: ExpressionDefinition {
            variables: vec![T],
            nodes: vec![ExpressionNode::Unary {
                operation: UnaryOperation::Neg,
                input: 0,
            }],
            outputs: vec![],
        },
        time: T,
        state: vec![],
    };
    assert!(matches!(
        ExpressionField::<0>::from_definition(invalid, &mut b()),
        Err(FieldError::Expression(
            ExpressionError::InvalidReference { .. }
        ))
    ));
}

#[test]
fn shape_bindings_and_source_references_are_validated_before_use() {
    let base = definition(&[variable(X), variable(V)], &[T, X, V], &[X, V]);
    let mut invalid = base.clone();
    invalid.state.pop();
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::StateCount {
            expected: 2,
            actual: 1
        })
    ));
    let mut invalid = base.clone();
    invalid.source.outputs.pop();
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::OutputCount {
            expected: 2,
            actual: 1
        })
    ));
    let mut invalid = base.clone();
    invalid.source.variables.pop();
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::BindingCount {
            expected: 3,
            actual: 2
        })
    ));
    let mut invalid = base.clone();
    invalid.state[0] = T;
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::TimeIsState(T))
    ));
    let mut invalid = base.clone();
    invalid.state[1] = X;
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::DuplicateState(X))
    ));
    let mut invalid = base.clone();
    invalid.time = SymbolId(99);
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::UnexpectedBinding(T))
    ));
    let mut invalid = base.clone();
    invalid.source.outputs[0] = usize::MAX;
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::Expression(
            ExpressionError::InvalidOutput { .. }
        ))
    ));
    let mut invalid = base;
    invalid.source.variables[0] = X;
    assert!(matches!(
        ExpressionField::<2>::from_definition(invalid, &mut b()),
        Err(FieldError::Expression(ExpressionError::DuplicateVariable(
            X
        )))
    ));
}

#[test]
fn policy_transitions_work_boundaries_cancellation_and_retry_preserve_identity() {
    let data = definition(&[variable(X).powi(2)], &[T, X], &[X]);
    let mut measured = b();
    let field = ExpressionField::<1>::from_definition(data.clone(), &mut measured).unwrap();
    let consumed = measured.policy().graph_work - measured.remaining_graph_work();
    let policy = EnclosurePolicy {
        graph_work: consumed,
        ..EnclosurePolicy::default()
    };
    let replay =
        ExpressionField::<1>::from_definition(data.clone(), &mut EnclosureBudget::new(policy))
            .unwrap();
    assert_eq!(field.identity(), replay.identity());
    assert!(matches!(
        ExpressionField::<1>::from_definition(
            data,
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_work: consumed - 1,
                ..policy
            })
        ),
        Err(FieldError::Expression(ExpressionError::WorkExhausted))
    ));
    let time = d(0, 1);
    let state = BoxN::new([d(1, 2)]).unwrap();
    let mut measured = b();
    let expected = field.jacobian(&time, &state, &mut measured).unwrap();
    let graph_work = measured.policy().graph_work - measured.remaining_graph_work();
    let scalar_work = measured.policy().scalar_work - measured.remaining_scalar_work();
    let exact = EnclosurePolicy {
        graph_work,
        scalar_work,
        ..EnclosurePolicy::default()
    };
    assert_eq!(
        field
            .jacobian(&time, &state, &mut EnclosureBudget::new(exact))
            .unwrap(),
        expected
    );
    assert!(matches!(
        field.jacobian(
            &time,
            &state,
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_work: graph_work - 1,
                ..exact
            })
        ),
        Err(FieldError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(matches!(
        field.jacobian(
            &time,
            &state,
            &mut EnclosureBudget::new(EnclosurePolicy {
                scalar_work: scalar_work - 1,
                ..exact
            })
        ),
        Err(FieldError::Expression(ExpressionError::Enclosure {
            source: IntervalError::WorkExhausted,
            ..
        })) | Err(FieldError::Scalar(IntervalError::WorkExhausted))
    ));
    // Construction and nested calls must not silently replenish either account.
    let mut image_budget = b();
    field.image(&time, &state, &mut image_budget).unwrap();
    let image_graph = image_budget.policy().graph_work - image_budget.remaining_graph_work();
    let image_scalar = image_budget.policy().scalar_work - image_budget.remaining_scalar_work();
    let aggregate = EnclosurePolicy {
        graph_work: image_graph + graph_work,
        scalar_work: image_scalar + scalar_work,
        ..EnclosurePolicy::default()
    };
    let mut nested = EnclosureBudget::new(aggregate);
    field.image(&time, &state, &mut nested).unwrap();
    assert_eq!(
        field.jacobian(&time, &state, &mut nested).unwrap(),
        expected
    );
    assert_eq!(nested.remaining_graph_work(), 0);
    assert_eq!(nested.remaining_scalar_work(), 0);
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: aggregate.graph_work - 1,
        ..aggregate
    });
    field.image(&time, &state, &mut short).unwrap();
    assert!(matches!(
        field.jacobian(&time, &state, &mut short),
        Err(FieldError::Expression(ExpressionError::WorkExhausted))
    ));
    let token = CancellationToken::new();
    let mut cancelled = b();
    cancelled.set_cancellation(Some(token.clone()));
    token.cancel();
    assert!(matches!(
        field.image(&DyadicInterval::empty(), &state, &mut cancelled),
        Err(FieldError::Expression(ExpressionError::Cancelled))
    ));
    assert!(matches!(
        field.image(
            &DyadicInterval::empty(),
            &state,
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_entries: 0,
                ..exact
            })
        ),
        Err(FieldError::Expression(ExpressionError::SizeExhausted))
    ));
    for precision in [-64, -8, 0] {
        assert_eq!(
            field
                .jacobian(
                    &time,
                    &state,
                    &mut EnclosureBudget::new(EnclosurePolicy {
                        precision,
                        ..EnclosurePolicy::default()
                    })
                )
                .unwrap(),
            expected
        );
    }
    assert_eq!(field.jacobian(&time, &state, &mut b()).unwrap(), expected);
}

#[test]
fn identity_includes_coordinate_order_and_remains_immutable_after_reconstruction() {
    let original = definition(
        &[variable(X), Expr::int(2) * variable(V)],
        &[V, T, X],
        &[X, V],
    );
    let field = ExpressionField::<2>::from_definition(original.clone(), &mut b()).unwrap();
    let old_identity = field.identity();
    let mut changed = original.clone();
    changed.state.reverse();
    let changed = ExpressionField::<2>::from_definition(changed, &mut b()).unwrap();
    assert_ne!(old_identity, changed.identity());
    assert_eq!(old_identity, field.clone().identity());
    assert_eq!(old_identity.source(), &original.source);
    assert_eq!(old_identity.time(), T);
    assert_eq!(old_identity.state(), [X, V]);
    let state = BoxN::new([d(3, 3), d(5, 5)]).unwrap();
    assert_eq!(
        field.image(&d(0, 0), &state, &mut b()).unwrap().lanes(),
        Some(&[d(3, 3), d(10, 10)])
    );
    assert_eq!(
        changed.image(&d(0, 0), &state, &mut b()).unwrap().lanes(),
        Some(&[d(5, 5), d(6, 6)])
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_revalidates_source_and_regenerates_field_partials() {
    const KEY: &str = "HYPERDYNAMICS_FIELD_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let definition: FieldDefinition = serde_json::from_str(&encoded).unwrap();
        let field = ExpressionField::<1>::from_definition(definition, &mut b()).unwrap();
        let state = BoxN::new([d(1, 2)]).unwrap();
        let image = field.image(&d(0, 1), &state, &mut b()).unwrap();
        let jacobian = field.jacobian(&d(0, 1), &state, &mut b()).unwrap();
        assert_eq!(jacobian.rows(), Some(&[[d(2, 4)]]));
        println!(
            "FIELD_REPLAY={}",
            serde_json::to_string(&(field.identity(), image, jacobian)).unwrap()
        );
        return;
    }
    let source = variable(X).powi(2) + Expr::real(Real::from(2).sqrt().unwrap());
    let field =
        ExpressionField::<1>::from_definition(definition(&[source], &[X, T], &[X]), &mut b())
            .unwrap();
    let encoded = serde_json::to_string(&field.identity()).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_revalidates_source_and_regenerates_field_partials",
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
    let output = String::from_utf8(child.stdout).unwrap();
    let row = output
        .lines()
        .find_map(|line| line.strip_prefix("FIELD_REPLAY="))
        .unwrap();
    let state = BoxN::new([d(1, 2)]).unwrap();
    let image = field.image(&d(0, 1), &state, &mut b()).unwrap();
    let jacobian = field.jacobian(&d(0, 1), &state, &mut b()).unwrap();
    assert_eq!(
        row,
        serde_json::to_string(&(field.identity(), image, jacobian)).unwrap()
    );
    let replay: FieldDefinition = serde_json::from_str(&encoded).unwrap();
    let replay = ExpressionField::<1>::from_definition(replay, &mut b()).unwrap();
    assert_eq!(field.identity(), replay.identity());
}
