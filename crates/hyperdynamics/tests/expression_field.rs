#![cfg(feature = "field")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    FieldError,
};
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalMatrix, IntervalVector};
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

#[test]
fn graph_adapter_shares_entry_limits_and_work_across_source_replay() {
    let policy = EnclosurePolicy {
        graph_entries: 2,
        graph_work: 8,
        ..EnclosurePolicy::default()
    };
    let mut budget = EnclosureBudget::new(policy);
    budget.graph_budget().check_entries(&[1, 1]).unwrap();
    assert!(matches!(
        budget.graph_budget().check_entries(&[1, 1, 1]),
        Err(ExpressionError::SizeExhausted)
    ));
    budget.graph_budget().charge(1).unwrap();
    // One output, two tree visits, then validation of one node and one output.
    let program = budget.graph_budget().compile(&[Expr::int(7)], &[]).unwrap();
    assert_eq!(budget.remaining_graph_work(), 2);
    let definition = program.definition();
    assert!(matches!(&definition.nodes[..], [ExpressionNode::Constant(v)] if *v == Real::from(7)));
    assert_eq!(definition.outputs, [0]);
    let stop = CancellationToken::new();
    stop.cancel();
    budget.set_cancellation(Some(stop));
    assert!(matches!(
        budget.graph_budget().validate(definition.clone()),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(budget.remaining_graph_work(), 2);
    budget.set_cancellation(None);
    let replay = budget.graph_budget().validate(definition.clone()).unwrap();
    assert_eq!(replay.definition(), definition);
    assert_eq!(budget.remaining_graph_work(), 0);
    assert!(matches!(
        budget.graph_budget().charge(1),
        Err(ExpressionError::WorkExhausted)
    ));
    assert_eq!(budget.remaining_scalar_work(), policy.scalar_work);

    let mut widest = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: usize::MAX,
        ..policy
    });
    widest
        .graph_budget()
        .check_entries(&[usize::MAX, 0])
        .unwrap();
    assert!(matches!(
        widest.graph_budget().check_entries(&[usize::MAX, 1]),
        Err(ExpressionError::SizeExhausted)
    ));
}

#[test]
fn rational_constants_obey_native_size_limits_and_retry_through_field_operations() {
    let exact = 1_000_000_000_000_i64;
    let field = ExpressionField::<1>::from_definition(
        definition(&[Expr::int(exact)], &[T, X], &[X]),
        &mut b(),
    )
    .unwrap();
    let time = d(0, 1);
    let state = BoxN::new([d(1, 2)]).unwrap();
    for jacobian in [false, true] {
        let mut small = EnclosureBudget::new(EnclosurePolicy {
            precision: 0,
            scalar_bits: 32,
            ..EnclosurePolicy::default()
        });
        let error = if jacobian {
            field.jacobian(&time, &state, &mut small).unwrap_err()
        } else {
            field.image(&time, &state, &mut small).unwrap_err()
        };
        assert!(matches!(
            error,
            FieldError::Expression(ExpressionError::Enclosure {
                source: IntervalError::SizeExhausted,
                ..
            })
        ));
    }
    assert_eq!(
        field.image(&time, &state, &mut b()).unwrap(),
        IntervalVector::new([d(exact, exact)]).unwrap()
    );
    assert_eq!(
        field.jacobian(&time, &state, &mut b()).unwrap(),
        IntervalMatrix::new([[d(0, 0)]]).unwrap()
    );
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

mod picard {
    use super::*;
    use hyperdynamics::field::picard::{PicardFailure, PicardProposal, PicardReport, PicardStatus};
    fn scaled(a: i64, b: i64, e: i32) -> DyadicInterval {
        DyadicInterval::new(a.into(), b.into(), e).unwrap()
    }
    fn certified<const N: usize>(report: &PicardReport<N>) {
        assert!(
            matches!(report.status(), PicardStatus::Certified),
            "{:?}",
            report.status()
        );
        assert!(report.end_enclosure().is_some());
        assert!(report.tube().is_some());
    }
    #[test]
    fn constant_flows_retain_uncertainty_and_compose_exact_endpoints() {
        for c in -3_i64..=3 {
            let field = ExpressionField::<1>::from_definition(
                definition(&[Expr::int(c)], &[T, X], &[X]),
                &mut b(),
            )
            .unwrap();
            for k in -3_i64..=3 {
                let candidate = BoxN::new([scaled(
                    (4 * k).min(4 * k + c),
                    (4 * k + 4).max(4 * k + 4 + c),
                    -2,
                )])
                .unwrap();
                let first = field.certify_step(
                    PicardProposal {
                        start: d(0, 0),
                        end: scaled(1, 1, -2),
                        initial: BoxN::new([d(k, k + 1)]).unwrap(),
                        candidate,
                    },
                    &mut b(),
                );
                certified(&first);
                assert_eq!(
                    first.end_enclosure().unwrap().lanes(),
                    Some(&[scaled(4 * k + c, 4 * k + 4 + c, -2)])
                );
                assert_eq!(first.tube().unwrap(), &first.proposal().candidate);
                assert_eq!(first.evidence().contraction(), Some(&d(0, 0)));
                let second = field.certify_step(
                    PicardProposal {
                        start: scaled(1, 1, -2),
                        end: scaled(1, 1, -1),
                        initial: first.end_enclosure().unwrap().clone(),
                        candidate: BoxN::new([scaled(
                            4 * k + c.min(2 * c),
                            4 * k + 4 + c.max(2 * c),
                            -2,
                        )])
                        .unwrap(),
                    },
                    &mut b(),
                );
                certified(&second);
                assert_eq!(
                    second.end_enclosure().unwrap().lanes(),
                    Some(&[scaled(2 * k + c, 2 * k + 2 + c, -1)])
                );
                assert_eq!(first.identity(), second.identity());
            }
        }
    }
    #[test]
    fn time_dependence_oscillator_and_riccati_have_independent_controls() {
        let field = ExpressionField::<1>::from_definition(
            definition(&[Expr::int(2) * variable(T) + Expr::int(3)], &[X, T], &[X]),
            &mut b(),
        )
        .unwrap();
        let report = field.certify_step(
            PicardProposal {
                start: d(0, 0),
                end: scaled(1, 1, -1),
                initial: BoxN::new([d(0, 0)]).unwrap(),
                candidate: BoxN::new([d(0, 2)]).unwrap(),
            },
            &mut b(),
        );
        certified(&report);
        assert_eq!(
            report.end_enclosure().unwrap().lanes(),
            Some(&[scaled(3, 4, -1)])
        );
        // Integral of 2t+3 from 0 to 1/2 is exactly 7/4.
        assert!(
            scaled(7, 7, -2)
                .subset_of(
                    &report.end_enclosure().unwrap().lanes().unwrap()[0],
                    &mut DyadicBudget::new(16384, 1_000_000_000)
                )
                .unwrap()
        );
        let coupled = ExpressionField::<2>::from_definition(
            definition(
                &[
                    variable(X) + Expr::int(2) * variable(V),
                    Expr::int(3) * variable(X) + Expr::int(4) * variable(V),
                ],
                &[V, T, X],
                &[X, V],
            ),
            &mut b(),
        )
        .unwrap();
        let coupled_report = coupled.certify_step(
            PicardProposal {
                start: d(0, 0),
                end: scaled(1, 1, -4),
                initial: BoxN::new([d(0, 0), d(0, 0)]).unwrap(),
                candidate: BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
            },
            &mut b(),
        );
        certified(&coupled_report);
        assert_eq!(
            coupled_report.evidence().jacobian().unwrap().rows(),
            Some(&[[d(1, 1), d(2, 2)], [d(3, 3), d(4, 4)]])
        );
        assert_eq!(coupled_report.evidence().lipschitz(), Some(&d(7, 7)));
        assert_eq!(
            coupled_report.evidence().contraction(),
            Some(&scaled(7, 7, -4))
        );
        let oscillator = ExpressionField::<2>::from_definition(
            definition(&[variable(V), -variable(X)], &[V, T, X], &[X, V]),
            &mut b(),
        )
        .unwrap();
        let report = oscillator.certify_step(
            PicardProposal {
                start: d(0, 0),
                end: scaled(1, 1, -2),
                initial: BoxN::new([d(1, 1), d(0, 0)]).unwrap(),
                candidate: BoxN::new([d(0, 2), d(-1, 1)]).unwrap(),
            },
            &mut b(),
        );
        certified(&report);
        assert_eq!(report.evidence().contraction(), Some(&scaled(1, 1, -2)));
        // y=(cos t,-sin t): 1-t²/2 <= cos t <=1, -t <= -sin t <=0 at t=1/4.
        let analytic = BoxN::new([scaled(31, 32, -5), scaled(-1, 0, -2)]).unwrap();
        assert!(
            analytic
                .subset_of(
                    report.end_enclosure().unwrap(),
                    &mut DyadicBudget::new(16384, 1_000_000_000)
                )
                .unwrap()
        );
        let riccati = ExpressionField::<1>::from_definition(
            definition(&[variable(X).powi(2)], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let proposal = PicardProposal {
            start: d(0, 0),
            end: scaled(1, 1, -3),
            initial: BoxN::new([d(1, 1)]).unwrap(),
            candidate: BoxN::new([d(1, 2)]).unwrap(),
        };
        let report = riccati.certify_step(proposal.clone(), &mut b());
        certified(&report);
        let exact = DyadicInterval::from_rational(
            &hyperreal::Rational::fraction(8, 7).unwrap(),
            -64,
            &mut DyadicBudget::new(16384, 1_000_000_000),
        )
        .unwrap();
        assert!(
            exact
                .subset_of(
                    &report.end_enclosure().unwrap().lanes().unwrap()[0],
                    &mut DyadicBudget::new(16384, 1_000_000_000)
                )
                .unwrap()
        );
        let mut blowup = proposal;
        blowup.end = d(1, 1);
        assert!(matches!(
            riccati.certify_step(blowup, &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::SelfMapNotCertified)
        ));
    }
    #[test]
    fn regularity_and_proposal_failures_do_not_become_trajectory_claims() {
        let time_root = ExpressionField::<1>::from_definition(
            definition(&[variable(T).sqrt()], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let proposal = PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([d(0, 0)]).unwrap(),
            candidate: BoxN::new([d(0, 1)]).unwrap(),
        };
        let report = time_root.certify_step(proposal.clone(), &mut b());
        certified(&report);
        // The exact temporal integral is 2/3 even though its time derivative is singular at zero.
        let two_thirds = DyadicInterval::from_rational(
            &hyperreal::Rational::fraction(2, 3).unwrap(),
            -64,
            &mut DyadicBudget::new(16384, 1_000_000_000),
        )
        .unwrap();
        assert!(
            two_thirds
                .subset_of(
                    &report.end_enclosure().unwrap().lanes().unwrap()[0],
                    &mut DyadicBudget::new(16384, 1_000_000_000)
                )
                .unwrap()
        );
        let state_root = ExpressionField::<1>::from_definition(
            definition(&[variable(X).sqrt()], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let rejected = state_root.certify_step(proposal.clone(), &mut b());
        assert!(matches!(
            rejected.status(),
            PicardStatus::Rejected(PicardFailure::Field(FieldError::Expression(
                ExpressionError::RegularityNotCertified { .. }
            )))
        ));
        assert!(rejected.end_enclosure().is_none());
        let linear = ExpressionField::<1>::from_definition(
            definition(&[variable(X)], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let mut p = proposal.clone();
        p.candidate = BoxN::new([d(-1, 1)]).unwrap();
        let rejected = linear.certify_step(p, &mut b());
        assert!(matches!(
            rejected.status(),
            PicardStatus::Rejected(PicardFailure::ContractionNotCertified)
        ));
        assert_eq!(rejected.evidence().contraction(), Some(&d(1, 1)));
        let constant = ExpressionField::<1>::from_definition(
            definition(&[Expr::int(1)], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let mut p = proposal.clone();
        p.candidate = BoxN::new([d(0, 0)]).unwrap();
        let rejected = constant.certify_step(p, &mut b());
        assert!(matches!(
            rejected.status(),
            PicardStatus::Rejected(PicardFailure::SelfMapNotCertified)
        ));
        assert_eq!(
            rejected.evidence().picard_image().unwrap().lanes(),
            Some(&[d(0, 1)])
        );
        assert!(rejected.tube().is_none());
        let pole = ExpressionField::<1>::from_definition(
            definition(&[Expr::int(1) / variable(X)], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        assert!(matches!(
            pole.certify_step(proposal, &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::Field(_))
        ));
    }
    #[test]
    fn empty_initial_and_zero_dimensional_unit_steps_remain_distinct() {
        let field =
            ExpressionField::<0>::from_definition(definition(&[], &[T], &[]), &mut b()).unwrap();
        let proposal = PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([]).unwrap(),
            candidate: BoxN::new([]).unwrap(),
        };
        let unit = field.certify_step(proposal.clone(), &mut b());
        certified(&unit);
        assert!(!unit.end_enclosure().unwrap().is_empty());
        let mut p = proposal.clone();
        p.initial = BoxN::empty().unwrap();
        p.candidate = BoxN::empty().unwrap();
        let empty = field.certify_step(p.clone(), &mut b());
        assert!(matches!(empty.status(), PicardStatus::Empty));
        assert!(empty.end_enclosure().unwrap().is_empty());
        p.start = DyadicInterval::empty();
        assert!(matches!(
            field.certify_step(p.clone(), &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::NonPointTime)
        ));
        p.start = d(0, 0);
        p.end = d(0, 0);
        assert!(matches!(
            field.certify_step(p, &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::NonPositiveDuration)
        ));
        let mut p = proposal;
        p.candidate = BoxN::empty().unwrap();
        assert!(matches!(
            field.certify_step(p, &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::InitialOutsideCandidate)
        ));
    }
    #[test]
    fn aggregate_limits_and_last_contribution_failure_precede_certification() {
        let field = ExpressionField::<1>::from_definition(
            definition(&[Expr::int(1)], &[T, X], &[X]),
            &mut b(),
        )
        .unwrap();
        let proposal = PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([d(0, 0)]).unwrap(),
            candidate: BoxN::new([d(0, 1)]).unwrap(),
        };
        let mut measured = b();
        certified(&field.certify_step(proposal.clone(), &mut measured));
        let policy = measured.policy();
        let gw = policy.graph_work - measured.remaining_graph_work();
        let sw = policy.scalar_work - measured.remaining_scalar_work();
        let exact = EnclosurePolicy {
            graph_work: gw,
            scalar_work: sw,
            ..policy
        };
        let mut work = EnclosureBudget::new(exact);
        certified(&field.certify_step(proposal.clone(), &mut work));
        assert_eq!(work.remaining_graph_work(), 0);
        assert_eq!(work.remaining_scalar_work(), 0);
        let graph_failure = field.certify_step(
            proposal.clone(),
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_work: gw - 1,
                ..policy
            }),
        );
        assert!(matches!(
            graph_failure.status(),
            PicardStatus::Rejected(PicardFailure::Field(FieldError::Expression(
                ExpressionError::WorkExhausted
            )))
        ));
        let last = field.certify_step(
            proposal.clone(),
            &mut EnclosureBudget::new(EnclosurePolicy {
                scalar_work: sw - 1,
                ..policy
            }),
        );
        assert!(
            last.evidence().end().is_some(),
            "last contribution must have been materialized"
        );
        assert!(last.end_enclosure().is_none());
        assert!(matches!(
            last.status(),
            PicardStatus::Rejected(PicardFailure::Field(_))
        ));
        let mut two = EnclosureBudget::new(EnclosurePolicy {
            graph_work: 2 * gw,
            scalar_work: 2 * sw,
            ..policy
        });
        certified(&field.certify_step(proposal.clone(), &mut two));
        certified(&field.certify_step(proposal.clone(), &mut two));
        assert_eq!(two.remaining_graph_work(), 0);
        assert_eq!(two.remaining_scalar_work(), 0);
        let token = CancellationToken::new();
        token.cancel();
        let mut cancelled = b();
        cancelled.set_cancellation(Some(token));
        assert!(matches!(
            field
                .certify_step(proposal.clone(), &mut cancelled)
                .status(),
            PicardStatus::Rejected(PicardFailure::Field(FieldError::Expression(
                ExpressionError::Cancelled
            )))
        ));
        certified(&field.certify_step(proposal, &mut b()));
    }
    #[cfg(feature = "serde")]
    #[test]
    fn picard_replay_reexecutes_source_and_candidate_validation_in_a_fresh_process() {
        fn signature(report: &PicardReport<1>) -> String {
            certified(report);
            let e = report.evidence();
            serde_json::to_string(&(
                report.identity(),
                report.proposal(),
                report.policy(),
                e.time(),
                e.duration(),
                e.image(),
                e.jacobian(),
                e.picard_image(),
                e.lipschitz(),
                e.contraction(),
                e.end(),
            ))
            .unwrap()
        }
        const KEY: &str = "HYPERDYNAMICS_PICARD_REPLAY";
        if let Ok(encoded) = std::env::var(KEY) {
            let (definition, proposal, policy): (
                FieldDefinition,
                PicardProposal<1>,
                EnclosurePolicy,
            ) = serde_json::from_str(&encoded).unwrap();
            let field = ExpressionField::<1>::from_definition(definition, &mut b()).unwrap();
            println!(
                "PICARD_REPLAY={}",
                signature(&field.certify_step(proposal, &mut EnclosureBudget::new(policy)))
            );
            return;
        }
        let source = definition(
            &[variable(X).powi(2) + Expr::real(Real::from(2).sqrt().unwrap())],
            &[X, T],
            &[X],
        );
        let field = ExpressionField::<1>::from_definition(source.clone(), &mut b()).unwrap();
        let proposal = PicardProposal {
            start: d(0, 0),
            end: scaled(1, 1, -4),
            initial: BoxN::new([d(0, 0)]).unwrap(),
            candidate: BoxN::new([d(0, 2)]).unwrap(),
        };
        let encoded =
            serde_json::to_string(&(field.identity(), &proposal, EnclosurePolicy::default()))
                .unwrap();
        let original = field.certify_step(proposal.clone(), &mut b());
        let expected = signature(&original);
        let child=std::process::Command::new(std::env::current_exe().unwrap()).args(["--exact","picard::picard_replay_reexecutes_source_and_candidate_validation_in_a_fresh_process","--nocapture"]).env(KEY,encoded).output().unwrap();
        assert!(
            child.status.success(),
            "{}",
            String::from_utf8_lossy(&child.stderr)
        );
        let stdout = String::from_utf8(child.stdout).unwrap();
        let actual = stdout
            .lines()
            .find_map(|l| l.strip_prefix("PICARD_REPLAY="))
            .unwrap();
        assert_eq!(actual, expected);
        let mut changed = proposal;
        changed.candidate = BoxN::new([d(0, 0)]).unwrap();
        assert!(matches!(
            field.certify_step(changed, &mut b()).status(),
            PicardStatus::Rejected(PicardFailure::SelfMapNotCertified)
        ));
        let mut invalid = source;
        invalid.source.outputs[0] = usize::MAX;
        assert!(ExpressionField::<1>::from_definition(invalid, &mut b()).is_err());
        assert_eq!(signature(&original), expected);
    }
}
