use super::*;
use hyperdynamics::field::{
    FieldError,
    coordinates::{CoordinateEvolution, CoordinateEvolutionStatus},
    evolution::{EvolutionFailure, EvolutionOptions, EvolutionRequest, EvolutionStatus},
};
use hyperinterval::{DyadicBudget, IntervalError};
use hypersolve::expression_program::ExpressionError;

fn request() -> EvolutionRequest<2> {
    EvolutionRequest {
        start: d(0, 0),
        end: d(3, 3),
        initial: BoxN::new([d(0, 0), d(-1, 1)]).unwrap(),
    }
}
fn options() -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(d(1, 1)),
        ..EvolutionOptions::default()
    }
}
fn no_source<const N: usize>(report: &CoordinateEvolution<N>) {
    assert!(report.source_initial_enclosure().is_none());
    assert!(report.source_end_enclosure().is_none());
    assert_eq!(report.segments().len(), 0);
}
fn constant_flow(report: &CoordinateEvolution<2>) {
    assert!(
        matches!(report.status(), CoordinateEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    assert_eq!(report.segments().len(), 3);
    assert_eq!(report.native().reached_time(), &d(3, 3));
    assert_eq!(
        report.source_initial_enclosure().unwrap().lanes(),
        Some(&[d(9, 11), d(-4, -2)])
    );
    // Independent analytic flow: x(t)=(10+b+2t,-3+b+t), b in [-1,1].
    for (index, segment) in report.segments().enumerate() {
        let t = i64::try_from(index + 1).unwrap();
        assert_eq!(segment.native().proposal().start, d(t - 1, t - 1));
        assert_eq!(segment.native().proposal().end, d(t, t));
        assert_eq!(
            segment.native().end_enclosure().unwrap().lanes(),
            Some(&[d(t, t), d(t - 1, t + 1)])
        );
        assert_eq!(
            segment.source_end_enclosure().lanes(),
            Some(&[d(9 + 2 * t, 11 + 2 * t), d(-4 + t, -2 + t)])
        );
        let tube = [d(9 + 2 * (t - 1), 11 + 2 * t), d(-4 + t - 1, -2 + t)];
        for (analytic, enclosed) in tube.iter().zip(segment.source_tube().lanes().unwrap()) {
            assert!(
                analytic
                    .subset_of(enclosed, &mut DyadicBudget::new(16384, 1_000_000_000))
                    .unwrap()
            );
        }
        assert_eq!(
            segment.chart().original_field().identity(),
            report.chart().original_field().identity()
        );
        assert_eq!(
            segment.native().identity(),
            &report.chart().transformed_field().identity()
        );
    }
    assert_eq!(
        report.source_end_enclosure().unwrap().lanes(),
        Some(&[d(15, 17), d(-1, 1)])
    );
}

#[test]
fn exact_endpoints_retained_correlation_and_subsequent_consumption() {
    let chart = chart();
    let report = chart.integrate(request(), options(), &mut budget());
    constant_flow(&report);
    // Every native attempt is retained verbatim, including rejected candidates.
    let direct = chart
        .transformed_field()
        .integrate(request(), options(), &mut budget());
    assert_eq!(format!("{:?}", report.native()), format!("{direct:?}"));
    assert!(report.native().attempts().len() > report.segments().len());
    // x_source-y_source=13+y_native[0]. The retained chart proves the point 16;
    // projecting first would only bound this difference by [14,18].
    let invariant = field::<2>(
        &[Expr::int(13) + Expr::anonymous_symbol(X), Expr::int(0)],
        vec![X, Y],
    );
    assert_eq!(
        invariant
            .image(
                &d(3, 3),
                report.native().end_enclosure().unwrap(),
                &mut budget()
            )
            .unwrap()
            .lanes(),
        Some(&[d(16, 16), d(0, 0)])
    );
    let next = report.chart().integrate(
        EvolutionRequest {
            start: d(3, 3),
            end: d(4, 4),
            initial: report.native().end_enclosure().unwrap().clone(),
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(
        next.status(),
        CoordinateEvolutionStatus::Completed
    ));
    assert_eq!(
        next.source_end_enclosure().unwrap().lanes(),
        Some(&[d(17, 19), d(0, 2)])
    );
}

#[test]
fn late_transport_failure_is_atomic_and_retry_uses_fresh_budget() {
    let chart = chart();
    let mut native_budget = budget();
    assert!(matches!(
        chart
            .transformed_field()
            .integrate(request(), options(), &mut native_budget)
            .status(),
        EvolutionStatus::Completed
    ));
    let mut measured = budget();
    constant_flow(&chart.integrate(request(), options(), &mut measured));
    let p = measured.policy();
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let ng = p.graph_work - native_budget.remaining_graph_work();
    let ns = p.scalar_work - native_budget.remaining_scalar_work();
    assert!(gw > ng && sw > ns);
    for (graph_work, scalar_work, scalar_failure) in [
        (ng, p.scalar_work, false),
        (gw - 1, p.scalar_work, false),
        (p.graph_work, ns, true),
        (p.graph_work, sw - 1, true),
    ] {
        let mut limited = EnclosureBudget::new(EnclosurePolicy {
            graph_work,
            scalar_work,
            ..p
        });
        let report = chart.integrate(request(), options(), &mut limited);
        assert!(matches!(
            report.native().status(),
            EvolutionStatus::Completed
        ));
        match report.status() {
            CoordinateEvolutionStatus::TransportFailed(CoordinateFieldError::Field(error)) => {
                if scalar_failure && scalar_work == ns {
                    assert!(
                        matches!(error, FieldError::Scalar(IntervalError::WorkExhausted)),
                        "scalar budget {scalar_work}: {error:?}"
                    );
                } else if scalar_failure {
                    assert!(matches!(
                        error,
                        FieldError::Lattice(hyperlattice_interval::IntervalLatticeError::Scalar(
                            IntervalError::WorkExhausted
                        ))
                    ));
                } else {
                    assert!(matches!(
                        error,
                        FieldError::Expression(ExpressionError::WorkExhausted)
                    ));
                }
            }
            other => panic!("unexpected status {other:?}"),
        }
        no_source(&report);
    }
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw,
        scalar_work: sw,
        ..p
    });
    constant_flow(&chart.integrate(request(), options(), &mut exact));
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    let mut twice = EnclosureBudget::new(EnclosurePolicy {
        graph_work: 2 * gw,
        scalar_work: 2 * sw,
        ..p
    });
    for _ in 0..2 {
        constant_flow(&chart.integrate(request(), options(), &mut twice));
    }
    assert_eq!(twice.remaining_graph_work(), 0);
    assert_eq!(twice.remaining_scalar_work(), 0);
    println!("WORK native_graph={ng} native_scalar={ns} complete_graph={gw} complete_scalar={sw}");
}

#[test]
fn native_prefix_and_cancellation_keep_original_diagnostics() {
    let chart = chart();
    let one = EvolutionOptions {
        maximum_steps: 1,
        ..options()
    };
    let mut direct_budget = budget();
    let direct = chart
        .transformed_field()
        .integrate(request(), one.clone(), &mut direct_budget);
    let mut wrapped_budget = budget();
    let report = chart.integrate(request(), one, &mut wrapped_budget);
    assert!(matches!(
        report.status(),
        CoordinateEvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 })
    ));
    assert_eq!(report.native().segments().len(), 1);
    assert!(report.native().prefix_end_enclosure().is_some());
    assert_eq!(format!("{:?}", report.native()), format!("{direct:?}"));
    assert_eq!(
        wrapped_budget.remaining_graph_work(),
        direct_budget.remaining_graph_work()
    );
    assert_eq!(
        wrapped_budget.remaining_scalar_work(),
        direct_budget.remaining_scalar_work()
    );
    no_source(&report);
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token));
    let report = chart.integrate(request(), options(), &mut cancelled);
    assert!(matches!(
        report.status(),
        CoordinateEvolutionStatus::Stopped(EvolutionFailure::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    no_source(&report);
    constant_flow(&chart.integrate(request(), options(), &mut budget()));
}

#[test]
fn output_entries_are_checked_together_before_transport_allocation() {
    let chart = chart();
    let req = EvolutionRequest {
        end: d(64, 64),
        ..request()
    };
    let native = chart
        .transformed_field()
        .integrate(req.clone(), options(), &mut budget());
    assert!(matches!(native.status(), EvolutionStatus::Completed));
    let n = native.segments().len();
    let entries = native.attempts().len() + 2 * n + 2 * (1 + 2 * n) + 4 + 6;
    let mut limited = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: entries - 1,
        ..EnclosurePolicy::default()
    });
    let report = chart.integrate(req.clone(), options(), &mut limited);
    assert!(
        matches!(report.native().status(), EvolutionStatus::Completed),
        "{:?}",
        report.native().status()
    );
    assert!(matches!(
        report.status(),
        CoordinateEvolutionStatus::TransportFailed(CoordinateFieldError::Field(
            FieldError::Expression(ExpressionError::SizeExhausted)
        ))
    ));
    no_source(&report);
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: entries,
        ..EnclosurePolicy::default()
    });
    let report = chart.integrate(req, options(), &mut exact);
    assert!(matches!(
        report.status(),
        CoordinateEvolutionStatus::Completed
    ));
    assert_eq!(report.segments().len(), 64);
    assert_eq!(
        report.source_end_enclosure().unwrap().lanes(),
        Some(&[d(137, 139), d(60, 62)])
    );
    println!("ENTRIES {entries}");
}

#[test]
fn identity_empty_unit_and_invalid_requests_remain_distinct() {
    let chart = chart();
    let identity = chart.integrate(
        EvolutionRequest {
            end: d(0, 0),
            ..request()
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(
        identity.status(),
        CoordinateEvolutionStatus::Identity
    ));
    assert_eq!(
        identity.source_initial_enclosure(),
        identity.source_end_enclosure()
    );
    assert_eq!(identity.segments().len(), 0);
    let empty = chart.integrate(
        EvolutionRequest {
            initial: BoxN::empty().unwrap(),
            ..request()
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(empty.status(), CoordinateEvolutionStatus::Empty));
    assert!(empty.source_end_enclosure().unwrap().is_empty());
    assert_eq!(empty.segments().len(), 0);
    for initial in [request().initial, BoxN::empty().unwrap()] {
        for (start, end, maximum_step, case) in [
            (d(4, 4), d(3, 3), d(1, 1), 0),
            (d(0, 1), d(3, 3), d(1, 1), 1),
            (d(0, 0), d(0, 0), d(0, 0), 2),
        ] {
            let report = chart.integrate(
                EvolutionRequest {
                    start,
                    end,
                    initial: initial.clone(),
                },
                EvolutionOptions {
                    maximum_step: Some(maximum_step),
                    ..options()
                },
                &mut budget(),
            );
            match (case, report.status()) {
                (0, CoordinateEvolutionStatus::Stopped(EvolutionFailure::ReversedTime))
                | (1, CoordinateEvolutionStatus::Stopped(EvolutionFailure::NonPointTime))
                | (2, CoordinateEvolutionStatus::Stopped(EvolutionFailure::InvalidMaximumStep)) => {
                }
                other => panic!("unexpected status {other:?}"),
            }
            no_source(&report);
        }
    }
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
    let report = unit.integrate(
        EvolutionRequest {
            start: d(0, 0),
            end: d(3, 3),
            initial: BoxN::new([]).unwrap(),
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(
        report.status(),
        CoordinateEvolutionStatus::Completed
    ));
    assert_eq!(report.segments().len(), 3);
    assert!(!report.source_end_enclosure().unwrap().is_empty());
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replays_chart_request_and_outputs() {
    const KEY: &str = "HYPERDYNAMICS_COORDINATE_EVOLUTION_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, req, opts): (
            CoordinateFieldDefinition,
            EvolutionRequest<2>,
            EvolutionOptions,
        ) = serde_json::from_str(&data).unwrap();
        let chart = CoordinateField::from_definition(definition, &mut budget()).unwrap();
        let report = chart.integrate(req, opts, &mut budget());
        constant_flow(&report);
        let rows: Vec<_> = report
            .segments()
            .map(|s| (s.source_tube(), s.source_end_enclosure()))
            .collect();
        println!(
            "EVOLUTION {}",
            serde_json::to_string(&(
                report.source_initial_enclosure(),
                rows,
                report.source_end_enclosure()
            ))
            .unwrap()
        );
        return;
    }
    let chart = chart();
    let identity = chart.original_field().identity();
    let definition = CoordinateFieldDefinition {
        source: FieldDefinition {
            source: identity.source().clone(),
            time: identity.time(),
            state: identity.state().to_vec(),
        },
        change: change(),
    };
    let data = serde_json::to_string(&(definition, request(), options())).unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "whole_horizon::fresh_process_replays_chart_request_and_outputs",
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
            .find(|s| s.starts_with("EVOLUTION "))
            .unwrap()
            .to_owned();
        if let Some(expected) = previous {
            assert_eq!(line, expected);
        }
        previous = Some(line);
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn native_and_transported_dispatch_counts_are_fixed() {
    use hyperreal::dispatch_trace;
    let chart = chart();
    dispatch_trace::reset();
    let report =
        dispatch_trace::with_recording(|| chart.integrate(request(), options(), &mut budget()));
    let trace = dispatch_trace::take_trace();
    constant_flow(&report);
    assert!(trace.dispatch.is_empty(), "{trace:?}");
    assert_eq!(
        trace.rational,
        dispatch_trace::RationalTraceStats::default()
    );
}
