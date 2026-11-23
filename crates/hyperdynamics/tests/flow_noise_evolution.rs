#![cfg(feature = "flow-noise-evolution")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::EvolutionOptions,
    taylor_flow::noise::{
        FlowNoiseState,
        evolution::{FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus},
    },
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::sync::Arc;

fn d(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}
fn shear(sign: i64, budget: &mut EnclosureBudget) -> ExpressionField<2> {
    let (time, x, y) = (SymbolId(10), SymbolId(20), SymbolId(30));
    let source = ExpressionProgram::compile(
        &[Expr::int(sign) * Expr::anonymous_symbol(y), Expr::int(0)],
        &[y, time, x],
        &mut ExpressionBudget::new(100_000, 1_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x, y],
        },
        budget,
    )
    .unwrap()
}
#[test]
fn automatic_horizons_preserve_reversible_shear_correlation_and_exact_time() {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let initial = BoxN::new([d(-1, 1, 0), d(-1, 1, 0)]).unwrap();
    let mut state =
        Arc::new(FlowNoiseState::new(d(0, 0, 0), initial.clone(), &mut budget).unwrap());
    for (end, sign) in [(3, 1), (6, -1)] {
        let program = shear(sign, &mut budget)
            .flow_taylor(1, &mut budget)
            .unwrap();
        let report = program.integrate_noise(
            state,
            d(end, end, -3),
            FlowNoiseEvolutionOptions {
                maximum_noise: 2,
                native: EvolutionOptions {
                    maximum_step: Some(d(1, 1, -2)),
                    ..Default::default()
                },
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
            "{:?}",
            report.status()
        );
        assert_eq!(report.segments().len(), 2);
        let next = report.end_state().unwrap().clone();
        assert_eq!(next.time(), &d(end, end, -3));
        assert!(next.evidence().is_some());
        for segment in report.segments() {
            assert!(segment.end_state().is_some());
        }
        state = next;
    }
    // Exact composition (x + 3*y/8) - 3*y/8 = x for every initial pair.
    assert_eq!(state.enclosure(), &initial);
}

#[test]
fn validation_precedes_empty_and_identity_and_empty_output_is_reusable() {
    use hyperdynamics::field::evolution::EvolutionFailure;
    use hyperdynamics::field::taylor_flow::noise::evolution::FlowNoiseEvolutionFailure;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let program = shear(1, &mut budget).flow_taylor(1, &mut budget).unwrap();
    let mut state =
        Arc::new(FlowNoiseState::new(d(0, 0, 0), BoxN::empty().unwrap(), &mut budget).unwrap());
    for end in [d(0, 1, 0), d(-1, -1, 0)] {
        let report = program.integrate_noise(state.clone(), end, Default::default(), &mut budget);
        assert!(matches!(
            report.status(),
            FlowNoiseEvolutionStatus::Stopped(_)
        ));
        assert!(report.end_state().is_none());
        assert_eq!(report.segments().len(), 0);
    }
    let invalid = program.integrate_noise(
        state.clone(),
        d(0, 0, 0),
        FlowNoiseEvolutionOptions {
            maximum_noise: 0,
            native: EvolutionOptions {
                maximum_step: Some(d(0, 0, 0)),
                ..Default::default()
            },
        },
        &mut budget,
    );
    assert!(matches!(
        invalid.status(),
        FlowNoiseEvolutionStatus::Stopped(FlowNoiseEvolutionFailure::Search(
            EvolutionFailure::InvalidMaximumStep
        ))
    ));
    assert!(invalid.end_state().is_none());
    for end in [0, 1, 2] {
        let report = program.integrate_noise(
            state.clone(),
            d(end, end, 0),
            Default::default(),
            &mut budget,
        );
        assert!(matches!(report.status(), FlowNoiseEvolutionStatus::Empty));
        assert_eq!(report.segments().len(), 0);
        let next = report.end_state().unwrap().clone();
        assert_eq!(next.time(), &d(end, end, 0));
        assert!(next.enclosure().is_empty());
        if end > 0 {
            assert!(Arc::ptr_eq(next.evidence().unwrap().initial(), &state));
        }
        state = next;
    }
    let point = Arc::new(
        FlowNoiseState::new(
            d(0, 0, 0),
            BoxN::new([d(0, 0, 0), d(1, 1, 0)]).unwrap(),
            &mut budget,
        )
        .unwrap(),
    );
    let report = program.integrate_noise(
        point.clone(),
        d(0, 0, 0),
        FlowNoiseEvolutionOptions {
            maximum_noise: 0,
            native: EvolutionOptions {
                maximum_steps: 0,
                ..Default::default()
            },
        },
        &mut budget,
    );
    assert!(matches!(
        report.status(),
        FlowNoiseEvolutionStatus::Identity
    ));
    assert!(Arc::ptr_eq(report.end_state().unwrap(), &point));
}

#[test]
fn stopped_prefix_is_explicit_and_can_seed_a_later_horizon() {
    use hyperdynamics::field::evolution::EvolutionFailure;
    use hyperdynamics::field::taylor_flow::noise::evolution::FlowNoiseEvolutionFailure;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let program = shear(1, &mut budget).flow_taylor(1, &mut budget).unwrap();
    let initial = Arc::new(
        FlowNoiseState::new(
            d(0, 0, 0),
            BoxN::new([d(0, 0, 0), d(1, 1, 0)]).unwrap(),
            &mut budget,
        )
        .unwrap(),
    );
    let options = FlowNoiseEvolutionOptions {
        maximum_noise: 2,
        native: EvolutionOptions {
            maximum_steps: 1,
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        },
    };
    let report = program.integrate_noise(initial, d(3, 3, -3), options, &mut budget);
    assert!(matches!(
        report.status(),
        FlowNoiseEvolutionStatus::Stopped(FlowNoiseEvolutionFailure::Search(
            EvolutionFailure::StepLimit { limit: 1 }
        ))
    ));
    assert!(report.end_state().is_none());
    assert_eq!(report.segments().len(), 1);
    let prefix = report.prefix_end_state().unwrap().clone();
    assert_eq!(prefix.time(), &d(1, 1, -2));
    let resumed = program.integrate_noise(prefix, d(3, 3, -3), Default::default(), &mut budget);
    assert!(
        matches!(resumed.status(), FlowNoiseEvolutionStatus::Completed),
        "{:?}",
        resumed.status()
    );
    assert_eq!(
        resumed.end_state().unwrap().enclosure().lanes(),
        Some(&[d(3, 3, -3), d(1, 1, 0)])
    );
}

#[test]
fn publication_exhaustion_cancellation_and_retry_never_publish_partial_success() {
    use hyperdynamics::field::taylor_flow::noise::FlowNoiseStatus;
    let mut setup = EnclosureBudget::new(EnclosurePolicy::default());
    let program = shear(1, &mut setup).flow_taylor(1, &mut setup).unwrap();
    let initial = Arc::new(
        FlowNoiseState::new(
            d(0, 0, 0),
            BoxN::new([d(-1, 1, 0), d(-1, 1, 0)]).unwrap(),
            &mut setup,
        )
        .unwrap(),
    );
    let options = FlowNoiseEvolutionOptions {
        maximum_noise: 2,
        native: EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        },
    };
    let run = |budget: &mut EnclosureBudget| {
        program.integrate_noise(initial.clone(), d(3, 3, -3), options.clone(), budget)
    };
    let policy = EnclosurePolicy::default();
    let mut ordinary = EnclosureBudget::new(policy);
    let complete = run(&mut ordinary);
    assert!(matches!(
        complete.status(),
        FlowNoiseEvolutionStatus::Completed
    ));
    let work = policy.graph_work - ordinary.remaining_graph_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: work - 1,
        ..policy
    });
    let failed = run(&mut short);
    assert!(matches!(
        failed.status(),
        FlowNoiseEvolutionStatus::Stopped(_)
    ));
    assert!(failed.end_state().is_none());
    assert_eq!(failed.segments().len(), 1);
    assert!(matches!(
        failed.attempts().last().unwrap().noise().unwrap().status(),
        FlowNoiseStatus::Certified
    ));
    assert_eq!(failed.prefix_end_state().unwrap().time(), &d(1, 1, -2));
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: work,
        ..policy
    });
    assert!(matches!(
        run(&mut exact).status(),
        FlowNoiseEvolutionStatus::Completed
    ));
    let mut zero = EnclosureBudget::new(EnclosurePolicy {
        graph_work: 0,
        ..policy
    });
    assert!(run(&mut zero).end_state().is_none());
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = EnclosureBudget::new(policy);
    cancelled.set_cancellation(Some(token));
    let failed = run(&mut cancelled);
    assert!(matches!(
        failed.status(),
        FlowNoiseEvolutionStatus::Stopped(_)
    ));
    assert!(failed.end_state().is_none());
    assert!(failed.attempts().is_empty());
    let retry = run(&mut EnclosureBudget::new(policy));
    assert_eq!(
        retry.end_state().unwrap().enclosure(),
        complete.end_state().unwrap().enclosure()
    );
    assert_eq!(initial.time(), &d(0, 0, 0));
    assert!(initial.evidence().is_none());
}

#[test]
fn zero_dimensional_unit_remains_nonempty_through_the_horizon() {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let time = SymbolId(10);
    let source =
        ExpressionProgram::compile(&[], &[time], &mut ExpressionBudget::new(1000, 10000)).unwrap();
    let field = ExpressionField::<0>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![],
        },
        &mut budget,
    )
    .unwrap();
    let program = field.flow_taylor(1, &mut budget).unwrap();
    let initial =
        Arc::new(FlowNoiseState::new(d(0, 0, 0), BoxN::new([]).unwrap(), &mut budget).unwrap());
    let report = program.integrate_noise(initial, d(1, 1, 0), Default::default(), &mut budget);
    assert!(
        matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    assert!(!report.end_state().unwrap().enclosure().is_empty());
    assert_eq!(report.end_state().unwrap().time(), &d(1, 1, 0));
}

#[test]
fn nonlinear_horizon_keeps_fresh_residuals_and_contains_the_exact_riccati_flow() {
    use hyperdynamics::field::taylor_flow::noise::FlowNoiseInput;
    use hyperinterval::DyadicBudget;
    use hyperreal::Rational;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let (time, x) = (SymbolId(10), SymbolId(20));
    let value = Expr::anonymous_symbol(x);
    let source = ExpressionProgram::compile(
        &[value.clone() * value],
        &[x, time],
        &mut ExpressionBudget::new(100000, 1000000),
    )
    .unwrap();
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x],
        },
        &mut budget,
    )
    .unwrap();
    let program = field.flow_taylor(2, &mut budget).unwrap();
    let initial = Arc::new(
        FlowNoiseState::new(d(0, 0, 0), BoxN::new([d(1, 2, -1)]).unwrap(), &mut budget).unwrap(),
    );
    let report = program.integrate_noise(
        initial,
        d(1, 1, -3),
        FlowNoiseEvolutionOptions {
            maximum_noise: 2,
            native: EvolutionOptions {
                maximum_step: Some(d(1, 1, -4)),
                ..Default::default()
            },
        },
        &mut budget,
    );
    assert!(
        matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    assert_eq!(report.segments().len(), 2);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
    for (index, segment) in report.segments().enumerate() {
        let e = segment.evidence();
        let next = segment.end_state().unwrap();
        assert_eq!(
            &e.flow().unwrap().picard().proposal().initial,
            e.initial().enclosure()
        );
        assert_eq!(e.flow().unwrap().evidence().end_terms().len(), 3);
        if index > 0 {
            assert!(e.inputs().contains(&FlowNoiseInput::Residual(0)));
        }
        let t = next
            .time()
            .to_rational(&mut arithmetic)
            .unwrap()
            .lower()
            .unwrap()
            .clone();
        let actual = next.enclosure().lanes().unwrap()[0]
            .to_rational(&mut arithmetic)
            .unwrap();
        // Monotone exact flow x0/(1-t*x0) on this initial interval and horizon.
        for x0 in [
            Rational::from(1) / Rational::from(2),
            Rational::from(3) / Rational::from(4),
            Rational::one(),
        ] {
            let exact = &x0 / (Rational::one() - &t * &x0);
            assert!(actual.contains_point(&exact), "{actual:?} misses {exact:?}");
        }
    }
}

#[test]
fn time_dependent_constant_state_derivative_is_integrated_exactly() {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let (time, x) = (SymbolId(10), SymbolId(20));
    let source = ExpressionProgram::compile(
        &[Expr::anonymous_symbol(time)],
        &[x, time],
        &mut ExpressionBudget::new(10000, 100000),
    )
    .unwrap();
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x],
        },
        &mut budget,
    )
    .unwrap();
    let program = field.flow_taylor(2, &mut budget).unwrap();
    let initial = Arc::new(
        FlowNoiseState::new(d(1, 1, -2), BoxN::new([d(0, 0, 0)]).unwrap(), &mut budget).unwrap(),
    );
    let report = program.integrate_noise(
        initial,
        d(3, 3, -2),
        FlowNoiseEvolutionOptions {
            maximum_noise: 0,
            native: EvolutionOptions {
                maximum_step: Some(d(1, 1, -2)),
                ..Default::default()
            },
        },
        &mut budget,
    );
    assert!(
        matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    // Integral from 1/4 to 3/4 of t dt is 1/4.
    assert_eq!(
        report.end_state().unwrap().enclosure().lanes(),
        Some(&[d(1, 1, -2)])
    );
}

#[cfg(feature = "serde")]
type Replay = (
    DyadicInterval,
    BoxN<DyadicInterval, 2>,
    Vec<(FieldDefinition, DyadicInterval, FlowNoiseEvolutionOptions)>,
);
#[cfg(feature = "serde")]
fn replay(data: Replay) -> Vec<u8> {
    let (start, initial, horizons) = data;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let mut state = Arc::new(FlowNoiseState::new(start, initial.clone(), &mut budget).unwrap());
    for (definition, end, options) in horizons {
        let field = ExpressionField::<2>::from_definition(definition, &mut budget).unwrap();
        let report = field.flow_taylor(1, &mut budget).unwrap().integrate_noise(
            state,
            end,
            options,
            &mut budget,
        );
        assert!(matches!(
            report.status(),
            FlowNoiseEvolutionStatus::Completed
        ));
        state = report.end_state().unwrap().clone();
    }
    assert_eq!(state.enclosure(), &initial);
    serde_json::to_vec(&(state.time(), state.enclosure())).unwrap()
}
#[test]
#[cfg(feature = "serde")]
fn fresh_process_reconstructs_sources_and_replays_automatic_horizons() {
    use hyperdynamics::field::EnclosedVectorField;
    const KEY: &str = "HYPERDYNAMICS_AUTOMATIC_NOISE_REPLAY";
    if let Ok(input) = std::env::var(KEY) {
        let data: Replay = serde_json::from_str(&input).unwrap();
        println!("REPLAY={}", String::from_utf8(replay(data)).unwrap());
        return;
    }
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let horizons = [(1, 3), (-1, 6)]
        .map(|(sign, end)| {
            let identity = shear(sign, &mut budget).identity();
            (
                FieldDefinition {
                    source: identity.source().clone(),
                    time: identity.time(),
                    state: identity.state().to_vec(),
                },
                d(end, end, -3),
                FlowNoiseEvolutionOptions {
                    maximum_noise: 2,
                    native: EvolutionOptions {
                        maximum_step: Some(d(1, 1, -2)),
                        ..Default::default()
                    },
                },
            )
        })
        .to_vec();
    let data = (
        d(0, 0, 0),
        BoxN::new([d(-1, 1, 0), d(-1, 1, 0)]).unwrap(),
        horizons,
    );
    let expected = String::from_utf8(replay(data.clone())).unwrap();
    let encoded = serde_json::to_string(&data).unwrap();
    for _ in 0..2 {
        let output = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_reconstructs_sources_and_replays_automatic_horizons",
                "--nocapture",
            ])
            .env(KEY, &encoded)
            .output()
            .unwrap();
        assert!(output.status.success(), "{output:?}");
        let stdout = String::from_utf8(output.stdout).unwrap();
        assert_eq!(
            stdout.lines().find_map(|line| line.strip_prefix("REPLAY=")),
            Some(expected.as_str())
        );
    }
}

#[path = "../examples/flow_noise_evolution.rs"]
mod shipped_example;
#[test]
fn shipped_automatic_noise_example_completes() {
    shipped_example::run().unwrap();
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
