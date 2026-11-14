#![cfg(feature = "coordinates")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{CoordinateField, CoordinateFieldError, CoordinateStatus},
    picard::{PicardProposal, PicardStatus},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
const T: SymbolId = SymbolId(9);
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);

#[path = "field_coordinates/whole_horizon.rs"]
mod whole_horizon;
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn field<const N: usize>(outputs: &[Expr], state: Vec<SymbolId>) -> ExpressionField<N> {
    let mut bindings = state.clone();
    bindings.reverse();
    bindings.push(T);
    let source = ExpressionProgram::compile(
        outputs,
        &bindings,
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state,
        },
        &mut budget(),
    )
    .unwrap()
}
fn change() -> CoordinateChange {
    CoordinateChange {
        variables: vec![X, Y],
        to_source: vec![vec![1.into(), 1.into()], vec![0.into(), 1.into()]],
        from_source: vec![vec![1.into(), (-1).into()], vec![0.into(), 1.into()]],
        origin: vec![10.into(), (-3).into()],
    }
}
fn chart() -> CoordinateField<2> {
    field(&[Expr::int(2), Expr::int(1)], vec![X, Y])
        .in_coordinates(change(), &mut budget())
        .unwrap()
}
fn proposal() -> PicardProposal<2> {
    PicardProposal {
        start: d(0, 0),
        end: d(1, 1),
        initial: BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
        candidate: BoxN::new([d(-1, 2), d(-1, 2)]).unwrap(),
    }
}
#[test]
fn correlated_initial_set_and_constant_flow_transport_keep_exact_chart() {
    let chart = chart();
    let step = chart.certify_step(proposal(), &mut budget());
    assert!(
        matches!(step.status(), CoordinateStatus::Certified),
        "{:?}",
        step.status()
    );
    assert_eq!(
        step.source_initial_enclosure().unwrap().lanes(),
        Some(&[d(8, 12), d(-4, -2)])
    );
    assert_eq!(
        step.source_tube().unwrap().lanes(),
        Some(&[d(8, 14), d(-4, -1)])
    );
    assert_eq!(
        step.source_end_enclosure().unwrap().lanes(),
        Some(&[d(10, 14), d(-3, -1)])
    );
    assert_eq!(
        step.native().identity(),
        &chart.transformed_field().identity()
    );
    assert_eq!(
        step.chart().original_field().identity(),
        chart.original_field().identity()
    );
    // Independent exact point trajectories x(t)=(10+a+b+2t,-3+b+t).
    for a in -1..=1 {
        for b in -1..=1 {
            let initial = BoxN::new([d(a, a), d(b, b)]).unwrap();
            let end = chart.certify_step(
                PicardProposal {
                    initial,
                    candidate: BoxN::new([d(a, a + 1), d(b, b + 1)]).unwrap(),
                    ..proposal()
                },
                &mut budget(),
            );
            assert_eq!(
                end.source_end_enclosure().unwrap().lanes(),
                Some(&[d(12 + a + b, 12 + a + b), d(-2 + b, -2 + b)])
            );
        }
    }
    // Explicit enclosure consumption in a subsequent field operation.
    let next = field::<1>(&[Expr::anonymous_symbol(X).powi(2)], vec![X]);
    let lane = step.source_end_enclosure().unwrap().lanes().unwrap()[1].clone();
    assert_eq!(
        next.image(&d(1, 1), &BoxN::new([lane]).unwrap(), &mut budget())
            .unwrap()
            .lanes(),
        Some(&[d(1, 9)])
    );
}
#[test]
fn transformed_nonlinear_values_and_jacobians_follow_exact_chain_rule() {
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    let t = Expr::anonymous_symbol(T);
    let source = field::<2>(
        &[x.clone().powi(2) + y.clone() + t, y.powi(2) - x],
        vec![X, Y],
    );
    let chart = source.in_coordinates(change(), &mut budget()).unwrap();
    for a in -2..=2 {
        for b in -2..=2 {
            let x = 10 + a + b;
            let y = b - 3;
            let state = BoxN::new([d(a, a), d(b, b)]).unwrap();
            assert_eq!(
                chart
                    .transformed_field()
                    .image(&d(2, 2), &state, &mut budget())
                    .unwrap()
                    .lanes(),
                Some(&[
                    d(x * x + y + 2 - y * y + x, x * x + y + 2 - y * y + x),
                    d(y * y - x, y * y - x)
                ])
            );
            let expected = [
                [
                    d(2 * x + 1, 2 * x + 1),
                    d(2 * x + 2 - 2 * y, 2 * x + 2 - 2 * y),
                ],
                [d(-1, -1), d(2 * y - 1, 2 * y - 1)],
            ];
            assert_eq!(
                chart
                    .transformed_field()
                    .jacobian(&d(2, 2), &state, &mut budget())
                    .unwrap()
                    .rows(),
                Some(&expected)
            );
        }
    }
    let mut wrong = change();
    wrong.variables.swap(0, 1);
    assert!(matches!(
        source.in_coordinates(wrong, &mut budget()),
        Err(CoordinateFieldError::StateOrder)
    ));
    let mut time = change();
    time.variables[0] = T;
    assert!(matches!(
        source.in_coordinates(time, &mut budget()),
        Err(CoordinateFieldError::StateOrder)
    ));
}
#[test]
fn native_success_cannot_hide_final_transport_exhaustion() {
    use hyperdynamics::field::FieldError;
    use hypersolve::expression_program::ExpressionError;
    use std::error::Error;
    let chart = chart();
    let mut measured = budget();
    let native = chart
        .transformed_field()
        .certify_step(proposal(), &mut measured);
    assert!(matches!(native.status(), PicardStatus::Certified));
    let native_work = EnclosurePolicy::default().graph_work - measured.remaining_graph_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: native_work,
        ..EnclosurePolicy::default()
    });
    let stopped = chart.certify_step(proposal(), &mut short);
    assert!(matches!(stopped.native().status(), PicardStatus::Certified));
    assert!(matches!(
        stopped.status(),
        CoordinateStatus::TransportFailed(CoordinateFieldError::Field(FieldError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    let CoordinateStatus::TransportFailed(error) = stopped.status() else {
        unreachable!()
    };
    assert!(
        error
            .source()
            .unwrap()
            .downcast_ref::<FieldError>()
            .is_some()
    );
    assert!(matches!(
        error
            .source()
            .unwrap()
            .source()
            .unwrap()
            .downcast_ref::<ExpressionError>(),
        Some(ExpressionError::WorkExhausted)
    ));
    assert!(stopped.source_end_enclosure().is_none());
    assert!(stopped.source_tube().is_none());
    let mut complete = budget();
    let good = chart.certify_step(proposal(), &mut complete);
    assert!(matches!(good.status(), CoordinateStatus::Certified));
    let used = EnclosurePolicy::default().graph_work - complete.remaining_graph_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        chart.certify_step(proposal(), &mut exact).status(),
        CoordinateStatus::Certified
    ));
    assert_eq!(exact.remaining_graph_work(), 0);
    let mut scalar = EnclosureBudget::new(EnclosurePolicy {
        scalar_work: 0,
        ..EnclosurePolicy::default()
    });
    let error = chart
        .source_enclosure(&proposal().initial, &mut scalar)
        .unwrap_err();
    assert!(matches!(
        error,
        CoordinateFieldError::Field(FieldError::Scalar(
            hyperinterval::IntervalError::WorkExhausted
        ))
    ));
    assert_eq!(
        error
            .source()
            .unwrap()
            .source()
            .unwrap()
            .downcast_ref::<hyperinterval::IntervalError>(),
        Some(&hyperinterval::IntervalError::WorkExhausted)
    );
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token));
    assert!(matches!(
        chart.certify_step(proposal(), &mut cancelled).status(),
        CoordinateStatus::Rejected(_)
    ));
    assert!(matches!(
        chart.certify_step(proposal(), &mut budget()).status(),
        CoordinateStatus::Certified
    ));
}
#[test]
fn empty_unit_and_invalid_time_keep_distinct_results() {
    let chart = chart();
    let empty = chart.certify_step(
        PicardProposal {
            initial: BoxN::empty().unwrap(),
            ..proposal()
        },
        &mut budget(),
    );
    assert!(matches!(empty.status(), CoordinateStatus::Empty));
    assert!(empty.source_end_enclosure().unwrap().is_empty());
    let invalid = chart.certify_step(
        PicardProposal {
            start: d(2, 2),
            end: d(1, 1),
            initial: BoxN::empty().unwrap(),
            ..proposal()
        },
        &mut budget(),
    );
    assert!(matches!(invalid.status(), CoordinateStatus::Rejected(_)));
    let unit = field::<0>(&[], vec![])
        .in_coordinates(
            CoordinateChange {
                variables: vec![],
                to_source: vec![],
                from_source: vec![],
                origin: vec![],
            },
            &mut budget(),
        )
        .unwrap();
    let step = unit.certify_step(
        PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([]).unwrap(),
            candidate: BoxN::new([]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(step.status(), CoordinateStatus::Certified),
        "{:?}",
        step.status()
    );
    assert!(!step.source_end_enclosure().unwrap().is_empty());
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_field_replay_preserves_chart_and_transported_step() {
    const KEY: &str = "HYPERDYNAMICS_COORDINATE_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let definition: CoordinateFieldDefinition = serde_json::from_str(&data).unwrap();
        let mut invalid = definition.clone();
        invalid.change.from_source[0][0] = 7.into();
        assert!(CoordinateField::<2>::from_definition(invalid, &mut budget()).is_err());
        let chart = CoordinateField::<2>::from_definition(definition, &mut budget()).unwrap();
        let result = chart.certify_step(proposal(), &mut budget());
        assert!(matches!(result.status(), CoordinateStatus::Certified));
        assert_eq!(
            result.source_end_enclosure().unwrap().lanes(),
            Some(&[d(10, 14), d(-3, -1)])
        );
        println!(
            "STEP {:?} {:?}",
            result.source_tube(),
            result.source_end_enclosure()
        );
        return;
    }
    let chart = chart();
    let source = chart.original_field().identity();
    let definition = CoordinateFieldDefinition {
        source: FieldDefinition {
            source: source.source().clone(),
            time: source.time(),
            state: source.state().to_vec(),
        },
        change: change(),
    };
    let data = serde_json::to_string(&definition).unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "fresh_process_field_replay_preserves_chart_and_transported_step",
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
            .find(|s| s.starts_with("STEP "))
            .unwrap()
            .to_owned();
        if let Some(expected) = previous {
            assert_eq!(line, expected);
        }
        previous = Some(line);
    }
}

#[test]
fn exact_scaling_certifies_a_step_rejected_in_original_coordinates() {
    use hyperdynamics::field::picard::PicardFailure;
    let original = field::<2>(
        &[Expr::int(128) * Expr::anonymous_symbol(Y), Expr::int(0)],
        vec![X, Y],
    );
    let half = DyadicInterval::point(1.into(), -1).unwrap();
    let native = original.certify_step(
        PicardProposal {
            start: d(0, 0),
            end: half.clone(),
            initial: BoxN::new([d(0, 0), d(-1, 1)]).unwrap(),
            candidate: BoxN::new([d(-128, 128), d(-1, 1)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(matches!(
        native.status(),
        PicardStatus::Rejected(PicardFailure::ContractionNotCertified)
    ));
    let chart = original
        .in_coordinates(
            CoordinateChange {
                variables: vec![X, Y],
                to_source: vec![vec![128.into(), 0.into()], vec![0.into(), 1.into()]],
                from_source: vec![
                    vec![
                        (hyperreal::Real::one() / hyperreal::Real::from(128)).unwrap(),
                        0.into(),
                    ],
                    vec![0.into(), 1.into()],
                ],
                origin: vec![0.into(), 0.into()],
            },
            &mut budget(),
        )
        .unwrap();
    let proof = chart.certify_step(
        PicardProposal {
            start: d(0, 0),
            end: half,
            initial: BoxN::new([d(0, 0), d(-1, 1)]).unwrap(),
            candidate: BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(proof.status(), CoordinateStatus::Certified),
        "{:?}",
        proof.status()
    );
    assert_eq!(
        proof.source_initial_enclosure(),
        Some(&BoxN::new([d(0, 0), d(-1, 1)]).unwrap())
    );
    // Exact flow: x1(t)=128*t*x2(0), x2(t)=x2(0).
    assert_eq!(
        proof.source_end_enclosure().unwrap().lanes(),
        Some(&[d(-64, 64), d(-1, 1)])
    );
    assert_eq!(
        proof.native().evidence().contraction(),
        Some(&DyadicInterval::point(1.into(), -1).unwrap())
    );
}

#[test]
fn source_projection_charges_coefficient_and_carrier_validation_scans() {
    fn check<const N: usize>() {
        let variables: Vec<_> = (0..N)
            .map(|i| SymbolId(u32::try_from(i).unwrap()))
            .collect();
        let matrix: Vec<Vec<_>> = (0..N)
            .map(|i| {
                (0..N)
                    .map(|j| hyperreal::Real::from(u8::from(i == j)))
                    .collect()
            })
            .collect();
        let source = field::<N>(&vec![Expr::int(0); N], variables.clone());
        let chart = source
            .in_coordinates(
                CoordinateChange {
                    variables,
                    to_source: matrix.clone(),
                    from_source: matrix,
                    origin: vec![0.into(); N],
                },
                &mut budget(),
            )
            .unwrap();
        let mut measured = budget();
        chart
            .source_enclosure(
                &BoxN::new(std::array::from_fn(|_| d(-1, 1))).unwrap(),
                &mut measured,
            )
            .unwrap();
        // N row visits, N² coefficient conversions, N translations, and
        // separate N²/N matrix/vector validation scans.
        assert_eq!(
            EnclosurePolicy::default().graph_work - measured.remaining_graph_work(),
            2 * N * N + 3 * N
        );
    }
    check::<0>();
    check::<1>();
    check::<3>();
}

#[cfg(feature = "serde")]
use hyperdynamics::field::coordinates::CoordinateFieldDefinition;
