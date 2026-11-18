#![cfg(feature = "taylor-evolution")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn field<const N: usize>(outputs: &[Expr], states: &[SymbolId]) -> ExpressionField<N> {
    let mut bindings = states.to_vec();
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
            state: states.to_vec(),
        },
        &mut budget(),
    )
    .unwrap()
}
#[test]
fn complete_taylor_horizon_consumes_each_certified_endpoint() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let p = f.flow_taylor(2, &mut budget()).unwrap();
    let report = p.integrate(
        EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([d(0, 0, 0)]).unwrap(),
        },
        EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        },
        &mut budget(),
    );
    assert_eq!(
        report.end_enclosure(),
        Some(&BoxN::new([d(1, 1, 0)]).unwrap())
    );
    assert_eq!(report.segments().len(), 2);
}

#[cfg(feature = "serde")]
use hyperdynamics::field::EnclosedVectorField;
use hyperdynamics::field::{
    evolution::{EvolutionFailure, EvolutionStatus},
    taylor_flow::{
        FlowTaylorError, FlowTaylorStatus,
        evolution::{FlowTaylorEvolutionFailure as Failure, FlowTaylorEvolutionStatus as Status},
    },
};
use hyperinterval::DyadicBudget;
fn request(initial: BoxN<DyadicInterval, 1>) -> EvolutionRequest<1> {
    EvolutionRequest {
        start: d(0, 0, 0),
        end: d(1, 1, -1),
        initial,
    }
}
fn options() -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(d(1, 1, -2)),
        ..Default::default()
    }
}
#[test]
fn nonautonomous_uncertain_flow_preserves_each_remainder_and_search() {
    let f = field::<1>(&[Expr::anonymous_symbol(T)], &[X]);
    let p = f.flow_taylor(2, &mut budget()).unwrap();
    let req = EvolutionRequest {
        end: d(1, 1, 0),
        ..request(BoxN::new([d(1, 2, 0)]).unwrap())
    };
    let report = p.integrate(req.clone(), options(), &mut budget());
    assert!(
        matches!(report.status(), Status::Completed),
        "{:?}",
        report.status()
    );
    assert_eq!(
        report.end_enclosure(),
        Some(&BoxN::new([d(3, 5, -1)]).unwrap())
    );
    assert_eq!(report.segments().len(), 4);
    let mut time = req.start;
    let mut initial = req.initial;
    for (i, segment) in report.segments().enumerate() {
        assert_eq!(segment.picard().proposal().start, time);
        assert_eq!(segment.picard().proposal().initial, initial);
        assert_eq!(segment.evidence().end_terms().len(), 3);
        assert_eq!(segment.evidence().tube_terms().len(), 3);
        assert!(segment.evidence().analytic().is_some());
        assert!(segment.evidence().remainder().is_some());
        assert!(
            initial
                .subset_of(segment.tube(), &mut DyadicBudget::new(16384, 1_000_000_000))
                .unwrap()
        );
        time = segment.picard().proposal().end.clone();
        initial = segment.end_enclosure().clone();
        let attempt = &report.attempts()[i];
        assert!(matches!(
            attempt.status(),
            Some(FlowTaylorStatus::Certified)
        ));
        if i < 3 {
            assert!(matches!(
                attempt.search().status(),
                EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 })
            ));
        }
        assert!(attempt.search().attempts().len() >= 2);
    }
    assert_eq!(time, d(1, 1, 0));
    assert_eq!(report.reached_time(), &time);
}
#[test]
fn exponential_endpoint_is_tighter_than_picard_and_contains_an_independent_series_bound() {
    use hyperreal::Rational as Q;
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap());
    let p = f.flow_taylor(3, &mut budget()).unwrap();
    let report = p.integrate(req.clone(), options(), &mut budget());
    let native = f.integrate(req, options(), &mut budget());
    assert!(
        matches!(report.status(), Status::Completed),
        "{:?}",
        report.status()
    );
    assert!(matches!(native.status(), EvolutionStatus::Completed));
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
    let end = report.end_enclosure().unwrap().lanes().unwrap()[0]
        .to_rational(&mut arithmetic)
        .unwrap();
    let mut term = Q::from(1);
    let mut lower = term.clone();
    for k in 1..=10 {
        term = term / Q::from(2 * k);
        lower = lower + &term;
    }
    let upper = &lower + (&term / Q::from(22)) / (Q::from(1) - Q::from(1) / Q::from(24));
    assert!(end.contains_point(&lower));
    assert!(end.contains_point(&upper));
    let widths = report
        .end_enclosure()
        .unwrap()
        .widths(&mut arithmetic)
        .unwrap()
        .unwrap();
    let old = native
        .end_enclosure()
        .unwrap()
        .widths(&mut arithmetic)
        .unwrap()
        .unwrap();
    assert_eq!(
        widths[0]
            .compare_singleton(&old[0], &mut arithmetic)
            .unwrap(),
        std::cmp::Ordering::Less
    );
    assert_eq!(
        widths[0]
            .compare_singleton(&d(1, 1, -9), &mut arithmetic)
            .unwrap(),
        std::cmp::Ordering::Less
    );
    let segments: Vec<_> = report.segments().collect();
    assert_eq!(segments.len(), 2);
    assert_eq!(
        &segments[1].picard().proposal().initial,
        segments[0].end_enclosure()
    );
    assert_ne!(
        segments[0].picard().end_enclosure(),
        Some(segments[0].end_enclosure())
    );
}
#[test]
fn empty_unit_identity_and_invalid_empty_requests_remain_distinct() {
    let p = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(2, &mut budget())
        .unwrap();
    let empty = request(BoxN::empty().unwrap());
    let r = p.integrate(empty.clone(), options(), &mut budget());
    assert!(matches!(r.status(), Status::Empty));
    assert!(r.end_enclosure().unwrap().is_empty());
    assert_eq!(r.reached_time(), &empty.end);
    assert_eq!(r.segments().len(), 0);
    for (req, opt) in [
        (
            EvolutionRequest {
                start: d(0, 1, 0),
                ..empty.clone()
            },
            options(),
        ),
        (
            EvolutionRequest {
                start: d(1, 1, 0),
                ..empty.clone()
            },
            options(),
        ),
        (
            empty.clone(),
            EvolutionOptions {
                maximum_step: Some(d(0, 0, 0)),
                ..options()
            },
        ),
    ] {
        assert!(matches!(
            p.integrate(req, opt, &mut budget()).status(),
            Status::Stopped(Failure::Search(
                EvolutionFailure::NonPointTime
                    | EvolutionFailure::ReversedTime
                    | EvolutionFailure::InvalidMaximumStep
            ))
        ));
    }
    let r = p.integrate(
        EvolutionRequest {
            end: d(0, 0, 0),
            ..request(BoxN::new([d(1, 2, 0)]).unwrap())
        },
        EvolutionOptions {
            maximum_steps: 0,
            ..options()
        },
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Identity));
    assert_eq!(r.end_enclosure(), Some(&BoxN::new([d(1, 2, 0)]).unwrap()));
    let p = field::<0>(&[], &[]).flow_taylor(2, &mut budget()).unwrap();
    let r = p.integrate(
        EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([]).unwrap(),
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    assert!(!r.end_enclosure().unwrap().is_empty());
    assert_eq!(r.segments().len(), 2);
}
#[test]
fn stopped_prefix_and_failed_analytic_refinement_are_never_complete_outputs() {
    let p = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(2, &mut budget())
        .unwrap();
    let req = request(BoxN::new([d(0, 0, 0)]).unwrap());
    let r = p.integrate(
        req.clone(),
        EvolutionOptions {
            maximum_steps: 1,
            ..options()
        },
        &mut budget(),
    );
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Search(EvolutionFailure::StepLimit { limit: 1 }))
    ));
    assert_eq!(r.segments().len(), 1);
    assert_eq!(r.reached_time(), &d(1, 1, -2));
    assert_eq!(
        r.prefix_end_enclosure(),
        Some(&BoxN::new([d(1, 1, -1)]).unwrap())
    );
    assert!(r.end_enclosure().is_none());
    let zero = p.integrate(
        req.clone(),
        EvolutionOptions {
            maximum_steps: 0,
            ..options()
        },
        &mut budget(),
    );
    assert!(matches!(
        zero.status(),
        Status::Stopped(Failure::Search(EvolutionFailure::StepLimit { limit: 0 }))
    ));
    assert_eq!(zero.segments().len(), 0);
    let p = field::<1>(&[Expr::anonymous_symbol(T).sqrt()], &[X])
        .flow_taylor(2, &mut budget())
        .unwrap();
    let r = p.integrate(req, options(), &mut budget());
    assert!(
        matches!(
            r.status(),
            Status::Stopped(Failure::Taylor(FlowTaylorError::Analytic(_)))
        ),
        "{:?}",
        r.status()
    );
    assert_eq!(r.attempts()[0].search().segments().len(), 1);
    assert!(matches!(
        r.attempts()[0].status(),
        Some(FlowTaylorStatus::Rejected(FlowTaylorError::Analytic(_)))
    ));
    assert!(r.end_enclosure().is_none());
    assert!(r.prefix_end_enclosure().is_none());
}
#[test]
fn final_work_limits_cancellation_and_retry_keep_completed_output_atomic() {
    let p = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(2, &mut budget())
        .unwrap();
    let req = request(BoxN::new([d(0, 0, 0)]).unwrap());
    let mut measured = budget();
    let full = p.integrate(req.clone(), options(), &mut measured);
    assert!(matches!(full.status(), Status::Completed));
    let work = EnclosurePolicy::default().graph_work - measured.remaining_graph_work();
    let scalar = EnclosurePolicy::default().scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: work,
        scalar_work: scalar,
        ..Default::default()
    });
    let r = p.integrate(req.clone(), options(), &mut exact);
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    for policy in [
        EnclosurePolicy {
            graph_work: work - 1,
            ..Default::default()
        },
        EnclosurePolicy {
            scalar_work: scalar - 1,
            ..Default::default()
        },
        EnclosurePolicy {
            graph_entries: 0,
            ..Default::default()
        },
    ] {
        let r = p.integrate(req.clone(), options(), &mut EnclosureBudget::new(policy));
        assert!(matches!(r.status(), Status::Stopped(_)), "{:?}", r.status());
        assert!(r.end_enclosure().is_none());
    }
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut stopped = budget();
    stopped.set_cancellation(Some(token));
    let r = p.integrate(req.clone(), options(), &mut stopped);
    assert!(matches!(r.status(), Status::Stopped(_)));
    assert!(r.attempts().is_empty());
    let retry = p.integrate(req, options(), &mut budget());
    assert_eq!(retry.end_enclosure(), full.end_enclosure());
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn immutable_evidence_access_does_not_repeat_decisions() {
    use hyperreal::dispatch_trace;
    let p = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(2, &mut budget())
        .unwrap();
    let r = p.integrate(
        request(BoxN::new([d(0, 0, 0)]).unwrap()),
        options(),
        &mut budget(),
    );
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        let _ = (
            r.program(),
            r.request(),
            r.options(),
            r.policy(),
            r.status(),
            r.reached_time(),
            r.prefix_end_enclosure(),
            r.end_enclosure(),
        );
        for a in r.attempts() {
            let _ = (a.search(), a.evidence(), a.status());
        }
        for a in r.segments() {
            let _ = (a.picard(), a.evidence(), a.tube(), a.end_enclosure());
        }
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[cfg(feature = "qr-coordinates")]
#[test]
fn qr_generated_flow_evolves_transports_and_continues() {
    use hyperreal::Real;
    let y = SymbolId(21);
    let f = field::<2>(&[Expr::int(2), Expr::int(1)], &[X, y]);
    let basis = vec![
        vec![Real::from(3), Real::from(-4)],
        vec![Real::from(4), Real::from(3)],
    ];
    let qr = f
        .in_qr_coordinates(basis.clone(), vec![0.into(), 0.into()], &mut budget())
        .unwrap();
    let p = qr
        .chart()
        .transformed_field()
        .flow_taylor(2, &mut budget())
        .unwrap();
    let r = p.integrate(
        EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([d(0, 0, 0), d(0, 0, 0)]).unwrap(),
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    let next = f
        .in_qr_coordinates(basis, vec![10.into(), (-3).into()], &mut budget())
        .unwrap();
    let transport = qr
        .chart()
        .transition_to(next.chart(), &mut budget())
        .unwrap()
        .enclose(r.end_enclosure().unwrap().clone(), &mut budget())
        .unwrap();
    let p = next
        .chart()
        .transformed_field()
        .flow_taylor(2, &mut budget())
        .unwrap();
    let r = p.integrate(
        EvolutionRequest {
            start: d(1, 1, -1),
            end: d(1, 1, 0),
            initial: transport.enclosure().clone(),
        },
        options(),
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    let output = next
        .chart()
        .source_enclosure(r.end_enclosure().unwrap(), &mut budget())
        .unwrap();
    let expected = BoxN::new([d(2, 2, 0), d(1, 1, 0)]).unwrap();
    let mut b = DyadicBudget::new(16384, 1_000_000_000);
    assert!(expected.subset_of(&output, &mut b).unwrap());
    for width in output.widths(&mut b).unwrap().unwrap() {
        assert_ne!(
            width.compare_singleton(&d(1, 1, -48), &mut b).unwrap(),
            std::cmp::Ordering::Greater
        );
    }
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_regenerates_all_search_and_taylor_evidence() {
    const KEY: &str = "HYPERDYNAMICS_TAYLOR_EVOLUTION_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, order, req, opt): (
            FieldDefinition,
            usize,
            EvolutionRequest<1>,
            EvolutionOptions,
        ) = serde_json::from_str(&data).unwrap();
        let mut work = budget();
        let f = ExpressionField::<1>::from_definition(definition, &mut work).unwrap();
        let p = f.flow_taylor(order, &mut work).unwrap();
        let r = p.integrate(req, opt, &mut work);
        assert!(matches!(r.status(), Status::Completed));
        println!(
            "REPLAY {:?} GRAPH {} SCALAR {}",
            r,
            work.remaining_graph_work(),
            work.remaining_scalar_work()
        );
        return;
    }
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    let id = f.identity();
    let definition = FieldDefinition {
        source: id.source().clone(),
        time: id.time(),
        state: id.state().to_vec(),
    };
    let data = serde_json::to_string(&(
        definition,
        3,
        request(BoxN::new([d(1, 1, 0)]).unwrap()),
        options(),
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_regenerates_all_search_and_taylor_evidence",
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
        let row = text
            .lines()
            .find(|l| l.starts_with("REPLAY "))
            .unwrap()
            .to_owned();
        if let Some(old) = previous {
            assert_eq!(row, old);
        }
        previous = Some(row);
    }
}

#[test]
fn adaptive_search_retains_halvings_and_original_failure_when_limit_is_reached() {
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    let p = f.flow_taylor(3, &mut budget()).unwrap();
    let req = EvolutionRequest {
        end: d(1, 1, 0),
        ..request(BoxN::new([d(1, 1, 0)]).unwrap())
    };
    let full = p.integrate(req.clone(), EvolutionOptions::default(), &mut budget());
    assert!(
        matches!(full.status(), Status::Completed),
        "{:?}",
        full.status()
    );
    assert!(
        full.attempts()
            .iter()
            .flat_map(|a| a.search().attempts())
            .any(|a| a.halvings() > 0)
    );
    let stopped = p.integrate(
        req,
        EvolutionOptions {
            maximum_halvings: 0,
            ..Default::default()
        },
        &mut budget(),
    );
    assert!(
        matches!(
            stopped.status(),
            Status::Stopped(Failure::Search(EvolutionFailure::HalvingLimit { limit: 0 }))
        ),
        "{:?}",
        stopped.status()
    );
    assert!(
        stopped.attempts()[0]
            .search()
            .attempts()
            .iter()
            .any(|a| matches!(
                a.report().status(),
                hyperdynamics::field::picard::PicardStatus::Rejected(_)
            ))
    );
    assert!(stopped.end_enclosure().is_none());
    assert!(stopped.attempts()[0].status().is_none());
}

#[test]
fn nested_search_records_count_toward_the_outer_entry_limit() {
    use hyperdynamics::field::FieldError;
    use hypersolve::expression_program::ExpressionError;
    let p = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(0, &mut budget())
        .unwrap();
    let req = EvolutionRequest {
        end: d(20, 20, 0),
        ..request(BoxN::new([d(0, 0, 0)]).unwrap())
    };
    let r = p.integrate(
        req.clone(),
        options(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            graph_entries: 128,
            ..Default::default()
        }),
    );
    assert!(
        matches!(
            r.status(),
            Status::Stopped(Failure::Search(EvolutionFailure::Field(
                FieldError::Expression(ExpressionError::SizeExhausted)
            )))
        ),
        "{:?}",
        r.status()
    );
    assert!(r.segments().len() > 1);
    let last = r.attempts().last().unwrap();
    assert_eq!(last.search().segments().len(), 1);
    assert!(last.status().is_none());
    assert!(r.end_enclosure().is_none());
    let count: usize = r
        .attempts()
        .iter()
        .map(|a| a.search().attempts().len() + a.search().segments().len())
        .sum();
    assert!(count + r.attempts().len() + r.segments().len() + 1 > 128);
    let retry = p.integrate(req, options(), &mut budget());
    assert!(
        matches!(retry.status(), Status::Completed),
        "{:?}",
        retry.status()
    );
    assert_eq!(
        retry.end_enclosure(),
        Some(&BoxN::new([d(40, 40, 0)]).unwrap())
    );
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
