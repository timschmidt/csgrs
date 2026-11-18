#![cfg(feature = "taylor-model")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor::FieldTaylorRequest,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn make_field<const N: usize>(
    outputs: &[Expr],
    bindings: &[SymbolId],
    state: &[SymbolId],
) -> ExpressionField<N> {
    let source = ExpressionProgram::compile(
        outputs,
        bindings,
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state: state.to_vec(),
        },
        &mut budget(),
    )
    .unwrap()
}

#[test]
fn logistic_field_retains_source_and_certifies_the_quadratic_range() {
    let x = Expr::anonymous_symbol(X);
    let field = make_field::<1>(&[x.clone() * (Expr::int(1) - x)], &[X, T], &[X]);
    let request = FieldTaylorRequest {
        time: d(0, 1, 0),
        state: BoxN::new([d(0, 1, 0)]).unwrap(),
        order: 2,
    };
    assert_eq!(
        field
            .image(&request.time, &request.state, &mut budget())
            .unwrap()
            .lanes(),
        Some(&[d(0, 1, 0)])
    );
    let model = field.taylor_model(request, &mut budget()).unwrap();
    assert_eq!(model.identity(), field.identity());
    assert_eq!(model.image().lanes(), Some(&[d(0, 1, -2)]));
    let polynomial = model.model().unwrap();
    assert_eq!(polynomial.source().definition(), field.identity().source());
    assert_eq!(polynomial.remainder(), &[d(0, 0, 0)]);
    assert_eq!(polynomial.request().center, vec![d(1, 1, -1), d(1, 1, -1)]);
    assert_eq!(
        model
            .enclose_on(
                &d(0, 0, 0),
                &BoxN::new([d(1, 1, -1)]).unwrap(),
                &mut budget()
            )
            .unwrap()
            .lanes(),
        Some(&[d(1, 1, -2)])
    );
}

#[test]
fn mixed_time_state_terms_and_native_output_composition_keep_the_binding_map() {
    const Y: SymbolId = SymbolId(30);
    let t = Expr::anonymous_symbol(T);
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    let field = make_field::<2>(
        &[t.clone() * x + y.clone(), t.powi(2) - y],
        &[Y, T, X],
        &[X, Y],
    );
    let request = FieldTaylorRequest {
        time: d(0, 2, 0),
        state: BoxN::new([d(1, 3, 0), d(3, 5, 0)]).unwrap(),
        order: 2,
    };
    let model = field.taylor_model(request, &mut budget()).unwrap();
    assert_eq!(
        model.model().unwrap().request().center,
        vec![d(4, 4, 0), d(1, 1, 0), d(2, 2, 0)]
    );
    assert_eq!(
        model.model().unwrap().remainder(),
        &[d(0, 0, 0), d(0, 0, 0)]
    );
    let point = BoxN::new([d(2, 2, 0), d(4, 4, 0)]).unwrap();
    let output = model
        .enclose_on(&d(1, 1, 0), &point, &mut budget())
        .unwrap();
    assert_eq!(output.lanes(), Some(&[d(6, 6, 0), d(-3, -3, 0)]));
    let consumer = make_field::<2>(
        &[
            Expr::anonymous_symbol(X) + Expr::anonymous_symbol(Y),
            Expr::anonymous_symbol(X) * Expr::anonymous_symbol(Y),
        ],
        &[T, X, Y],
        &[X, Y],
    );
    assert_eq!(
        consumer
            .image(&d(0, 0, 0), &output, &mut budget())
            .unwrap()
            .lanes(),
        Some(&[d(3, 3, 0), d(-18, -18, 0)])
    );
    assert!(matches!(
        model.enclose_on(&d(3, 3, 0), &point, &mut budget()),
        Err(hyperdynamics::field::taylor::FieldTaylorError::Taylor(
            hypersolve::expression_program::TaylorError::OutsideDomain { slot: 1 }
        ))
    ));
    assert!(
        model
            .enclose_on(&d(100, 100, 0), &BoxN::empty().unwrap(), &mut budget())
            .unwrap()
            .is_empty()
    );
}

#[test]
fn empty_unit_original_guards_and_stronger_regularity_are_distinct() {
    use hyperdynamics::field::taylor::FieldTaylorError;
    let unit = make_field::<0>(&[], &[T], &[]);
    for (time, state, empty) in [
        (d(0, 1, 0), BoxN::new([]).unwrap(), false),
        (DyadicInterval::empty(), BoxN::new([]).unwrap(), true),
        (d(0, 1, 0), BoxN::empty().unwrap(), true),
    ] {
        let model = unit
            .taylor_model(
                FieldTaylorRequest {
                    time,
                    state,
                    order: usize::MAX,
                },
                &mut budget(),
            )
            .unwrap();
        assert_eq!(model.image().is_empty(), empty);
        assert_eq!(model.model().is_none(), empty);
        assert!(
            model
                .enclose_on(&d(10, 11, 0), &BoxN::empty().unwrap(), &mut budget())
                .unwrap()
                .is_empty()
        );
        let reused = model.enclose_on(&d(0, 0, 0), &BoxN::new([]).unwrap(), &mut budget());
        if empty {
            assert!(matches!(reused, Err(FieldTaylorError::EmptyDomain)));
        } else {
            assert!(!reused.unwrap().is_empty());
        }
    }
    let x = Expr::anonymous_symbol(X);
    let guarded = make_field::<1>(&[Expr::int(0) * x.clone().sqrt()], &[T, X], &[X]);
    let request = FieldTaylorRequest {
        time: d(0, 1, 0),
        state: BoxN::new([d(-2, -1, 0)]).unwrap(),
        order: 2,
    };
    assert!(guarded.taylor_model(request, &mut budget()).is_err());
    let root = make_field::<1>(&[x.sqrt()], &[T, X], &[X]);
    let request = FieldTaylorRequest {
        time: d(0, 1, 0),
        state: BoxN::new([d(0, 1, 0)]).unwrap(),
        order: 2,
    };
    assert!(
        root.image(&request.time, &request.state, &mut budget())
            .is_ok()
    );
    assert!(matches!(
        root.taylor_model(request, &mut budget()),
        Err(FieldTaylorError::Taylor(
            hypersolve::expression_program::TaylorError::Expression(
                hypersolve::expression_program::ExpressionError::RegularityNotCertified { .. }
            )
        ))
    ));
    let mut zero_scalar = EnclosureBudget::new(EnclosurePolicy {
        scalar_bits: 0,
        scalar_work: 0,
        ..EnclosurePolicy::default()
    });
    let model = guarded
        .taylor_model(
            FieldTaylorRequest {
                time: DyadicInterval::empty(),
                state: BoxN::new([d(-2, -1, 0)]).unwrap(),
                order: 2,
            },
            &mut zero_scalar,
        )
        .unwrap();
    assert!(model.image().is_empty());
    assert_eq!(zero_scalar.remaining_scalar_work(), 0);
}

#[test]
fn cumulative_budget_boundaries_cancellation_and_policy_reuse_retain_evidence() {
    use hypersolve::work_budget::CancellationToken;
    let x = Expr::anonymous_symbol(X);
    let field = make_field::<1>(&[x.powi(2)], &[T, X], &[X]);
    let request = FieldTaylorRequest {
        time: d(0, 1, 0),
        state: BoxN::new([d(-1, 1, 0)]).unwrap(),
        order: 2,
    };
    let mut measured = budget();
    let model = field.taylor_model(request.clone(), &mut measured).unwrap();
    let graph_work = measured.policy().graph_work - measured.remaining_graph_work();
    let scalar_work = measured.policy().scalar_work - measured.remaining_scalar_work();
    let exact = EnclosurePolicy {
        graph_work,
        scalar_work,
        ..EnclosurePolicy::default()
    };
    let completed = field
        .taylor_model(request.clone(), &mut EnclosureBudget::new(exact))
        .unwrap();
    assert_eq!(format!("{model:?}"), format!("{completed:?}"));
    for policy in [
        EnclosurePolicy {
            graph_work: 0,
            ..exact
        },
        EnclosurePolicy {
            graph_work: graph_work - 1,
            ..exact
        },
        EnclosurePolicy {
            scalar_work: 0,
            ..exact
        },
        EnclosurePolicy {
            scalar_work: scalar_work - 1,
            ..exact
        },
        EnclosurePolicy {
            graph_entries: 0,
            ..exact
        },
        EnclosurePolicy {
            scalar_bits: 0,
            ..exact
        },
    ] {
        let error = field
            .taylor_model(request.clone(), &mut EnclosureBudget::new(policy))
            .unwrap_err();
        assert!(std::error::Error::source(&error).is_some(), "{error:?}");
    }
    let time = d(0, 0, 0);
    let state = BoxN::new([d(1, 1, -1)]).unwrap();
    let before = (
        measured.remaining_graph_work(),
        measured.remaining_scalar_work(),
    );
    let image = model.enclose_on(&time, &state, &mut measured).unwrap();
    assert_eq!(image.lanes(), Some(&[d(1, 1, -2)]));
    let reuse = (
        before.0 - measured.remaining_graph_work(),
        before.1 - measured.remaining_scalar_work(),
    );
    let mut aggregate = EnclosureBudget::new(EnclosurePolicy {
        graph_work: graph_work + reuse.0,
        scalar_work: scalar_work + reuse.1,
        ..exact
    });
    let rebuilt = field.taylor_model(request.clone(), &mut aggregate).unwrap();
    assert_eq!(
        rebuilt.enclose_on(&time, &state, &mut aggregate).unwrap(),
        image
    );
    assert_eq!(
        (
            aggregate.remaining_graph_work(),
            aggregate.remaining_scalar_work()
        ),
        (0, 0)
    );
    for precision in [0, -8, -64] {
        assert_eq!(
            model
                .enclose_on(
                    &time,
                    &state,
                    &mut EnclosureBudget::new(EnclosurePolicy { precision, ..exact })
                )
                .unwrap(),
            image
        );
    }
    let token = CancellationToken::new();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token.clone()));
    token.cancel();
    let error = field
        .taylor_model(request.clone(), &mut cancelled)
        .unwrap_err();
    assert!(format!("{error:?}").contains("Cancelled"));
    assert!(
        model
            .enclose_on(&DyadicInterval::empty(), &state, &mut cancelled)
            .is_err()
    );
    assert_eq!(
        format!("{:?}", field.taylor_model(request, &mut budget()).unwrap()),
        format!("{model:?}")
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_field_interpretation_and_remainder() {
    const KEY: &str = "HYPERDYNAMICS_TAYLOR_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, request): (FieldDefinition, FieldTaylorRequest<1>) =
            serde_json::from_str(&encoded).unwrap();
        let mut shared = budget();
        let field = ExpressionField::<1>::from_definition(definition, &mut shared).unwrap();
        let model = field.taylor_model(request, &mut shared).unwrap();
        assert_eq!(model.image().lanes(), Some(&[d(0, 1, -2)]));
        println!(
            "TAYLOR_FIELD_REPLAY={model:?};graph={};scalar={}",
            shared.remaining_graph_work(),
            shared.remaining_scalar_work()
        );
        return;
    }
    let x = Expr::anonymous_symbol(X);
    let field = make_field::<1>(&[x.clone() * (Expr::int(1) - x)], &[X, T], &[X]);
    let request = FieldTaylorRequest {
        time: d(0, 1, 0),
        state: BoxN::new([d(0, 1, 0)]).unwrap(),
        order: 2,
    };
    let encoded = serde_json::to_string(&(field.identity(), request)).unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_rebuilds_field_interpretation_and_remainder",
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
            .find(|line| line.starts_with("TAYLOR_FIELD_REPLAY="))
            .unwrap()
            .to_owned();
        if let Some(previous) = previous {
            assert_eq!(previous, row);
        }
        previous = Some(row);
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
