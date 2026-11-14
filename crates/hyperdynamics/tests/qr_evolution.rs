#![cfg(feature = "qr-evolution")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{
        CoordinateField, QrFieldError,
        qr_evolution::{
            QrEvolution, QrEvolutionFailure as Failure, QrEvolutionOptions,
            QrEvolutionStatus as Status,
        },
    },
    evolution::{EvolutionFailure, EvolutionOptions, EvolutionRequest},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        CoordinateChange, ExpressionBudget, ExpressionProgram, QrCoordinateError,
    },
};
#[path = "support/flow_oracles.rs"]
mod flow_oracles;
use flow_oracles::alternating_trig;
const T: SymbolId = SymbolId(9);
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn field<const N: usize>(outputs: &[Expr], state: &[SymbolId]) -> ExpressionField<N> {
    let mut bindings = state.to_vec();
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
            state: state.to_vec(),
        },
        &mut budget(),
    )
    .unwrap()
}
fn identity<const N: usize>(f: &ExpressionField<N>) -> CoordinateField<N> {
    let a: Vec<Vec<Real>> = (0..N)
        .map(|i| (0..N).map(|j| u8::from(i == j).into()).collect())
        .collect();
    f.in_coordinates(
        CoordinateChange {
            variables: f.identity().state().to_vec(),
            to_source: a.clone(),
            from_source: a,
            origin: vec![Real::zero(); N],
        },
        &mut budget(),
    )
    .unwrap()
}
fn request<const N: usize>(
    initial: [DyadicInterval; N],
    end: DyadicInterval,
) -> EvolutionRequest<N> {
    EvolutionRequest {
        start: d(0, 0, 0),
        end,
        initial: BoxN::new(initial).unwrap(),
    }
}
fn options(window: DyadicInterval) -> QrEvolutionOptions {
    QrEvolutionOptions {
        window,
        maximum_windows: 8,
        taylor_order: Some(3),
        native: EvolutionOptions {
            maximum_step: Some(d(1, 1, -4)),
            ..Default::default()
        },
    }
}
fn completed<const N: usize>(r: &QrEvolution<N>) {
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    assert_eq!(r.reached_time(), &r.request().end);
    assert_eq!(r.completed_windows().len(), r.windows().len());
    assert!(r.source_end_enclosure().is_some());
    for w in r.completed_windows() {
        let comparison = w.comparison().unwrap();
        let proposed = w.proposed().unwrap().source_end_enclosure().unwrap();
        let retained = w.retained().unwrap().source_end_enclosure().unwrap();
        let width = |b: &BoxN<DyadicInterval, N>| {
            b.lanes()
                .unwrap()
                .iter()
                .map(|x| x.to_rational(&mut arithmetic()).unwrap().width().unwrap())
                .fold(Rational::zero(), |a, b| if b > a { b } else { a })
        };
        let ordering = width(proposed)
            .partial_cmp(&width(retained))
            .expect("finite rational widths");
        assert_eq!(comparison.ordering(), ordering);
        let choice = if ordering == std::cmp::Ordering::Greater {
            hyperdynamics::field::coordinates::qr_evolution::QrFrameChoice::Retained
        } else {
            hyperdynamics::field::coordinates::qr_evolution::QrFrameChoice::Proposed
        };
        assert_eq!(comparison.choice(), choice);
        for branch in [w.proposed().unwrap(), w.retained().unwrap()] {
            assert_eq!(
                branch.evolution().unwrap().request().end,
                *w.evolution().unwrap().reached_time()
            );
            assert!(branch.evolution().unwrap().end_enclosure().is_some());
        }
        assert_eq!(
            w.frame().unwrap().chart().original_field().identity(),
            r.chart().original_field().identity()
        );
        assert_eq!(w.transport().unwrap().initial(), w.selection().initial());
        assert_eq!(
            &w.proposed().unwrap().evolution().unwrap().request().initial,
            w.transport().unwrap().enclosure()
        );
        assert_eq!(
            w.frame().unwrap().factorization().request().basis,
            w.selection().basis()
        );
        assert_eq!(
            w.frame().unwrap().factorization().request().origin,
            w.selection().origin()
        );
    }
    for pair in r.completed_windows().windows(2) {
        assert_eq!(
            pair[1].selection().initial(),
            pair[0].evolution().unwrap().end_enclosure().unwrap()
        );
        let a = pair[1].selection().chart().transformation().change();
        let b = pair[0].chosen().unwrap().chart().transformation().change();
        assert_eq!(a.variables, b.variables);
        assert_eq!(a.to_source, b.to_source);
        assert_eq!(a.from_source, b.from_source);
        assert_eq!(a.origin, b.origin);
    }
}
#[test]
fn constant_flow_recenters_and_caps_final_window_then_consumes_output() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let chart = identity(&f);
    let r = chart.integrate_qr(
        request([d(-1, 1, 0)], d(5, 5, -2)),
        options(d(1, 1, -1)),
        &mut budget(),
    );
    completed(&r);
    assert_eq!(r.windows().len(), 3);
    assert_eq!(
        r.source_end_enclosure().unwrap().lanes(),
        Some(&[d(3, 7, -1)])
    );
    for (w, (origin, duration)) in
        r.windows()
            .iter()
            .zip([(0, d(1, 1, -1)), (1, d(1, 1, -1)), (2, d(1, 1, -2))])
    {
        assert_eq!(w.selection().basis(), &[vec![Real::one()]]);
        assert_eq!(w.selection().origin(), &[Real::from(origin)]);
        assert_eq!(w.selection().duration(), &duration);
    }
    let last = r.windows().last().unwrap();
    let second = last.chosen().unwrap().chart().integrate_qr(
        EvolutionRequest {
            start: r.reached_time().clone(),
            end: d(3, 3, -1),
            initial: last.evolution().unwrap().end_enclosure().unwrap().clone(),
        },
        options(d(1, 1, -2)),
        &mut budget(),
    );
    completed(&second);
    assert_eq!(
        second.source_end_enclosure().unwrap().lanes(),
        Some(&[d(2, 4, 0)])
    );
}
#[test]
fn correlated_authored_chart_keeps_original_image_and_transport_proof() {
    let f = field::<2>(&[Expr::int(2), Expr::int(1)], &[X, Y]);
    let chart = f
        .in_coordinates(
            CoordinateChange {
                variables: vec![X, Y],
                to_source: vec![vec![1.into(), 1.into()], vec![0.into(), 1.into()]],
                from_source: vec![vec![1.into(), (-1).into()], vec![0.into(), 1.into()]],
                origin: vec![10.into(), (-3).into()],
            },
            &mut budget(),
        )
        .unwrap();
    let r = chart.integrate_qr(
        request([d(0, 0, 0), d(-1, 1, 0)], d(1, 1, 0)),
        options(d(1, 1, -1)),
        &mut budget(),
    );
    completed(&r);
    // Original correlated source is (10+b,-3+b), -1<=b<=1; flow is (2t,t).
    assert_eq!(
        r.source_end_enclosure().unwrap().lanes(),
        Some(&[d(11, 13, 0), d(-3, -1, 0)])
    );
    assert_eq!(
        r.windows()[0].selection().basis(),
        &[
            vec![Real::one(), Real::one()],
            vec![Real::zero(), Real::one()]
        ]
    );
    assert_ne!(
        r.windows()[0].transport().unwrap().initial(),
        r.windows()[0].transport().unwrap().enclosure()
    );
}
#[test]
fn rotation_frames_follow_exact_basis_relations_and_independent_trigonometric_bounds() {
    let f = field::<2>(
        &[-Expr::anonymous_symbol(Y), Expr::anonymous_symbol(X)],
        &[X, Y],
    );
    let chart = identity(&f);
    for (horizon, window, expect) in [
        (d(3, 3, -1), d(3, 3, -2), [(7073, 7074), (99749, 99750)]),
        (d(1, 1, -1), d(1, 1, -2), [(87758, 87759), (47942, 47943)]),
    ] {
        let t = horizon
            .to_rational(&mut arithmetic())
            .unwrap()
            .lower()
            .unwrap()
            .clone();
        let oracle = [alternating_trig(&t, false), alternating_trig(&t, true)];
        let r = chart.integrate_qr(
            request([d(1, 1, 0), d(0, 0, 0)], horizon),
            options(window),
            &mut budget(),
        );
        completed(&r);
        // Alternating Taylor series give these independent rational brackets for cos/sin.
        for ((lane, (l, u)), (analytic_lower, analytic_upper)) in r
            .source_end_enclosure()
            .unwrap()
            .lanes()
            .unwrap()
            .iter()
            .zip(expect)
            .zip(oracle)
        {
            let interval = lane.to_rational(&mut arithmetic()).unwrap();
            // A certified endpoint can be narrower than the displayed decimal bracket.
            // Verify that bracket independently, then consume the tighter series oracle.
            assert!(analytic_lower >= Rational::from(l) / Rational::from(100000));
            assert!(analytic_upper <= Rational::from(u) / Rational::from(100000));
            assert!(
                interval.contains_point(&analytic_lower),
                "lower {:?} oracle {:?}",
                interval.lower().unwrap().to_f64_enclosure(),
                analytic_lower.to_f64_enclosure()
            );
            assert!(interval.contains_point(&analytic_upper));
            eprintln!(
                "ROTATION width {:?} steps {:?}",
                interval.width().unwrap().to_f64_enclosure(),
                r.windows()
                    .iter()
                    .map(|w| w.source_segments().len())
                    .collect::<Vec<_>>()
            );
            assert!(interval.width().unwrap() < Rational::one());
        }
        assert_eq!(r.windows().len(), 2);
        let first = r.windows()[0].selection();
        let h = first
            .duration()
            .to_rational(&mut arithmetic())
            .unwrap()
            .lower()
            .unwrap()
            .clone();
        assert_eq!(
            first.basis(),
            &[
                vec![Real::one(), Real::from(-&h)],
                vec![Real::from(h), Real::one()]
            ]
        );
    }
}
#[test]
fn singular_frame_and_original_domain_failure_are_not_identity_fallbacks() {
    let f = field::<1>(&[-Expr::anonymous_symbol(X)], &[X]);
    let r = identity(&f).integrate_qr(
        request([d(1, 1, 0)], d(1, 1, 0)),
        options(d(1, 1, 0)),
        &mut budget(),
    );
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Qr(QrFieldError::Qr(
            QrCoordinateError::DependentColumn { .. }
        )))
    ));
    assert_eq!(r.windows().len(), 1);
    assert_eq!(r.windows()[0].selection().basis(), &[vec![Real::zero()]]);
    assert!(r.windows()[0].frame().is_none());
    assert!(r.source_end_enclosure().is_none());
    let f = field::<1>(&[Expr::int(1) / Expr::anonymous_symbol(X)], &[X]);
    let r = identity(&f).integrate_qr(
        request([d(-1, 1, 0)], d(1, 1, 0)),
        options(d(1, 1, -2)),
        &mut budget(),
    );
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Native(EvolutionFailure::Field(_)))
    ));
    assert!(r.windows().is_empty());
    assert!(r.source_end_enclosure().is_none());
}
#[test]
fn empty_unit_identity_invalid_inputs_and_limits_stay_distinct() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let chart = identity(&f);
    let o = options(d(1, 1, -1));
    let mut input = request([d(0, 0, 0)], d(1, 1, 0));
    input.initial = BoxN::empty().unwrap();
    let r = chart.integrate_qr(input.clone(), o.clone(), &mut budget());
    assert!(matches!(r.status(), Status::Empty));
    assert_eq!(r.reached_time(), &d(1, 1, 0));
    assert!(r.source_end_enclosure().unwrap().is_empty());
    let mut bad = o.clone();
    bad.window = d(0, 0, 0);
    assert!(matches!(
        chart
            .integrate_qr(input.clone(), bad, &mut budget())
            .status(),
        Status::Stopped(Failure::InvalidWindow)
    ));
    input.start = d(2, 2, 0);
    assert!(matches!(
        chart.integrate_qr(input, o.clone(), &mut budget()).status(),
        Status::Stopped(Failure::Native(EvolutionFailure::ReversedTime))
    ));
    let r = chart.integrate_qr(request([d(-1, 1, 0)], d(0, 0, 0)), o.clone(), &mut budget());
    assert!(matches!(r.status(), Status::Identity));
    assert_eq!(
        r.source_end_enclosure().unwrap().lanes(),
        Some(&[d(-1, 1, 0)])
    );
    let f = field::<0>(&[], &[]);
    let r = identity(&f).integrate_qr(request([], d(1, 1, 0)), o.clone(), &mut budget());
    completed(&r);
    assert!(
        r.source_end_enclosure()
            .unwrap()
            .lanes()
            .unwrap()
            .is_empty()
    );
    let mut limited = o.clone();
    limited.maximum_windows = 1;
    let r = chart.integrate_qr(request([d(0, 0, 0)], d(1, 1, 0)), limited, &mut budget());
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::WindowLimit { limit: 1 })
    ));
    assert_eq!(r.completed_windows().len(), 1);
    assert_eq!(r.reached_time(), &d(1, 1, -1));
    assert_eq!(
        r.prefix_source_end_enclosure().unwrap().lanes(),
        Some(&[d(1, 1, 0)])
    );
    assert!(r.source_end_enclosure().is_none());
    let mut limited = o;
    limited.taylor_order = None;
    limited.native.maximum_steps = 0;
    let r = chart.integrate_qr(request([d(0, 0, 0)], d(1, 1, 0)), limited, &mut budget());
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Native(EvolutionFailure::StepLimit { limit: 0 }))
    ));
    assert!(r.windows()[0].proposed().unwrap().evolution().is_some());
    assert!(r.completed_windows().is_empty());
    assert!(r.source_end_enclosure().is_none());
}

#[test]
fn native_and_taylor_choices_keep_the_same_exact_constant_flow() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let chart = identity(&f);
    for order in [None, Some(0), Some(3)] {
        let mut o = options(d(1, 1, -2));
        o.taylor_order = order;
        let r = chart.integrate_qr(request([d(-1, 1, 0)], d(1, 1, -1)), o, &mut budget());
        completed(&r);
        assert_eq!(
            r.source_end_enclosure().unwrap().lanes(),
            Some(&[d(0, 2, 0)])
        );
        for window in r.windows() {
            assert_eq!(window.program().map(|p| p.order()), order);
        }
    }
}
#[test]
fn small_budgets_final_publication_cancellation_and_retry_preserve_typed_failures() {
    use hyperdynamics::field::FieldError;
    use hypersolve::expression_program::ExpressionError;
    let chart = identity(&field::<1>(&[Expr::int(2)], &[X]));
    let input = request([d(0, 0, 0)], d(1, 1, -2));
    let o = options(d(1, 1, -2));
    let mut b = budget();
    let result = chart.integrate_qr(input.clone(), o.clone(), &mut b);
    completed(&result);
    let used = b.policy().graph_work - b.remaining_graph_work();
    let scalar = b.policy().scalar_work - b.remaining_scalar_work();
    assert!(used > 1 && scalar > 1);
    // Exhaust every early structural-work cutoff and the complete-path boundary.
    for work in (0..used.min(512)).chain([used / 2, used - 1]) {
        let mut b = EnclosureBudget::new(EnclosurePolicy {
            graph_work: work,
            ..EnclosurePolicy::default()
        });
        let r = chart.integrate_qr(input.clone(), o.clone(), &mut b);
        assert!(
            matches!(r.status(), Status::Stopped(_)),
            "work={work} {:?}",
            r.status()
        );
        assert!(
            format!("{:?}", r.status()).contains("WorkExhausted"),
            "{:?}",
            r.status()
        );
        assert!(r.source_end_enclosure().is_none());
    }
    let mut b = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used,
        scalar_work: scalar,
        ..EnclosurePolicy::default()
    });
    let r = chart.integrate_qr(input.clone(), o.clone(), &mut b);
    completed(&r);
    assert_eq!(b.remaining_graph_work(), 0);
    assert_eq!(b.remaining_scalar_work(), 0);
    let mut b = EnclosureBudget::new(EnclosurePolicy {
        scalar_work: scalar - 1,
        ..EnclosurePolicy::default()
    });
    let r = chart.integrate_qr(input.clone(), o.clone(), &mut b);
    assert!(matches!(r.status(), Status::Stopped(_)));
    assert!(r.source_end_enclosure().is_none());
    for entries in 0..8 {
        let mut b = EnclosureBudget::new(EnclosurePolicy {
            graph_entries: entries,
            ..EnclosurePolicy::default()
        });
        let r = chart.integrate_qr(input.clone(), o.clone(), &mut b);
        assert!(matches!(r.status(), Status::Stopped(_)));
        assert!(r.source_end_enclosure().is_none());
    }
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut b = budget();
    b.set_cancellation(Some(token));
    let r = chart.integrate_qr(input.clone(), o.clone(), &mut b);
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Native(EvolutionFailure::Field(
            FieldError::Expression(ExpressionError::Cancelled)
        )))
    ));
    assert!(r.windows().is_empty());
    assert!(r.source_end_enclosure().is_none());
    let retried = chart.integrate_qr(input, o, &mut budget());
    completed(&retried);
    assert_eq!(
        retried.source_end_enclosure(),
        result.source_end_enclosure()
    );
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replays_original_chart_request_options_and_all_source_outputs() {
    use hyperdynamics::field::coordinates::CoordinateFieldDefinition;
    const KEY: &str = "HYPERDYNAMICS_QR_EVOLUTION_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, input, o): (
            CoordinateFieldDefinition,
            EvolutionRequest<1>,
            QrEvolutionOptions,
        ) = serde_json::from_str(&data).unwrap();
        let chart = CoordinateField::from_definition(definition, &mut budget()).unwrap();
        let result = chart.integrate_qr(input, o, &mut budget());
        completed(&result);
        assert_eq!(
            result.source_end_enclosure().unwrap().lanes(),
            Some(&[d(0, 2, 0)])
        );
        let rows: Vec<_> = result
            .windows()
            .iter()
            .map(|w| {
                (
                    w.selection().basis(),
                    w.selection().origin(),
                    w.source_segments(),
                )
            })
            .collect();
        println!(
            "QR_EVOLUTION {}",
            serde_json::to_string(&(result.source_end_enclosure(), rows)).unwrap()
        );
        return;
    }
    let chart = identity(&field::<1>(&[Expr::int(2)], &[X]));
    let id = chart.original_field().identity();
    let definition = CoordinateFieldDefinition {
        source: FieldDefinition {
            source: id.source().clone(),
            time: id.time(),
            state: id.state().to_vec(),
        },
        change: chart.transformation().change().clone(),
    };
    let data = serde_json::to_string(&(
        definition,
        request([d(-1, 1, 0)], d(1, 1, -1)),
        options(d(1, 1, -2)),
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "fresh_process_replays_original_chart_request_options_and_all_source_outputs",
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
            .find(|x| x.starts_with("QR_EVOLUTION "))
            .unwrap()
            .to_owned();
        if let Some(expected) = previous {
            assert_eq!(row, expected);
        }
        previous = Some(row);
    }
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn evidence_getters_do_not_repeat_decisions_or_scalar_work() {
    use hyperreal::dispatch_trace;
    let chart = identity(&field::<1>(&[Expr::int(2)], &[X]));
    let r = chart.integrate_qr(
        request([d(0, 0, 0)], d(1, 1, -2)),
        options(d(1, 1, -2)),
        &mut budget(),
    );
    completed(&r);
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        std::hint::black_box((
            r.chart(),
            r.request(),
            r.options(),
            r.policy(),
            r.status(),
            r.reached_time(),
            r.source_end_enclosure(),
            r.prefix_source_end_enclosure(),
        ));
        for w in r.windows() {
            let s = w.selection();
            std::hint::black_box((
                s.chart(),
                s.time(),
                s.duration(),
                s.initial(),
                s.midpoint(),
                s.jacobian(),
                s.basis(),
                s.origin(),
            ));
            std::hint::black_box((
                w.frame(),
                w.transition(),
                w.transport(),
                w.program(),
                w.source_segments(),
                w.source_end_enclosure(),
            ));
            let c = w.comparison().unwrap();
            std::hint::black_box((
                c.proposed_widths(),
                c.retained_widths(),
                c.proposed_diameter(),
                c.retained_diameter(),
                c.ordering(),
                c.choice(),
            ));
            for branch in [w.proposed().unwrap(), w.retained().unwrap()] {
                std::hint::black_box((
                    branch.chart(),
                    branch.program(),
                    branch.evolution(),
                    branch.source_segments(),
                    branch.source_end_enclosure(),
                ));
            }
            let e = w.evolution().unwrap();
            std::hint::black_box((e.request(), e.reached_time(), e.end_enclosure()));
        }
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[test]
fn uncertain_rotating_line_contains_the_entire_independent_analytic_image() {
    let f = field::<2>(
        &[-Expr::anonymous_symbol(Y), Expr::anonymous_symbol(X)],
        &[X, Y],
    );
    let chart = identity(&f);
    let r = chart.integrate_qr(
        request([d(-1, 1, 0), d(0, 0, 0)], d(3, 3, -1)),
        options(d(3, 3, -2)),
        &mut budget(),
    );
    completed(&r);
    assert!(r.windows().iter().any(|w| w.comparison().unwrap().choice()
        == hyperdynamics::field::coordinates::qr_evolution::QrFrameChoice::Retained));
    let t = Rational::from(3) / Rational::from(2);
    for (lane, sine) in r
        .source_end_enclosure()
        .unwrap()
        .lanes()
        .unwrap()
        .iter()
        .zip([false, true])
    {
        let (lower, upper) = alternating_trig(&t, sine);
        assert!(lower.is_positive());
        let q = lane.to_rational(&mut arithmetic()).unwrap();
        assert!(q.contains_point(&(-&upper)));
        assert!(q.contains_point(&upper));
        eprintln!(
            "UNCERTAIN width {:?} choices {:?}",
            q.width().unwrap().to_f64_enclosure(),
            r.windows()
                .iter()
                .map(|w| w.comparison().unwrap().choice())
                .collect::<Vec<_>>()
        );
        assert!(q.width().unwrap() < Rational::from(10));
    }
}
#[test]
fn all_invalid_controls_are_checked_before_empty_identity_shortcuts() {
    let chart = identity(&field::<1>(&[Expr::int(2)], &[X]));
    let mut input = request([d(0, 0, 0)], d(1, 1, 0));
    input.initial = BoxN::empty().unwrap();
    for bad in [
        DyadicInterval::empty(),
        d(-1, -1, 0),
        d(0, 1, 0),
        d(0, 0, 0),
    ] {
        let mut o = options(d(1, 1, -1));
        o.window = bad.clone();
        let r = chart.integrate_qr(input.clone(), o, &mut budget());
        assert!(matches!(
            r.status(),
            Status::Stopped(Failure::InvalidWindow)
        ));
        assert!(r.source_end_enclosure().is_none());
        let mut o = options(d(1, 1, -1));
        o.native.maximum_step = Some(bad);
        let r = chart.integrate_qr(input.clone(), o, &mut budget());
        assert!(matches!(
            r.status(),
            Status::Stopped(Failure::Native(EvolutionFailure::InvalidMaximumStep))
        ));
        assert!(r.source_end_enclosure().is_none());
    }
    let mut bad = input.clone();
    bad.start = d(0, 1, 0);
    assert!(matches!(
        chart
            .integrate_qr(bad, options(d(1, 1, -1)), &mut budget())
            .status(),
        Status::Stopped(Failure::Native(EvolutionFailure::NonPointTime))
    ));
    let mut o = options(d(1, 1, -1));
    o.taylor_order = Some(usize::MAX);
    assert!(matches!(
        chart.integrate_qr(input, o, &mut budget()).status(),
        Status::Stopped(Failure::Taylor(
            hyperdynamics::field::taylor_flow::FlowTaylorError::OrderOverflow
        ))
    ));
}
#[test]
fn selected_frame_certifies_longer_steps_with_exact_equilibrium_output() {
    use hyperdynamics::field::coordinates::qr_evolution::{QrFrameChoice, QrWindowEvolution};
    let q = |n: i64| Expr::real(Real::from(Rational::from(n) / Rational::from(25)));
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    let chart = identity(&field::<2>(
        &[q(7) * x.clone() + q(24) * y.clone(), q(24) * x + q(-7) * y],
        &[X, Y],
    ));
    let mut o = options(d(7, 7, -3));
    o.native.maximum_step = Some(d(7, 7, -3));
    let input = request([d(0, 0, 0), d(0, 0, 0)], d(7, 7, -2));
    let fixed = chart
        .transformed_field()
        .flow_taylor(3, &mut budget())
        .unwrap()
        .integrate(input.clone(), o.native.clone(), &mut budget());
    assert!(matches!(
        fixed.status(),
        hyperdynamics::field::taylor_flow::evolution::FlowTaylorEvolutionStatus::Completed
    ));
    assert_eq!(fixed.segments().len(), 4);
    let r = chart.integrate_qr(input, o, &mut budget());
    completed(&r);
    assert_eq!(
        r.source_end_enclosure().unwrap().lanes(),
        Some(&[d(0, 0, 0), d(0, 0, 0)])
    );
    for w in r.windows() {
        assert_eq!(w.comparison().unwrap().choice(), QrFrameChoice::Proposed);
        let QrWindowEvolution::Taylor(flow) = w.evolution().unwrap() else {
            panic!("Taylor strategy")
        };
        assert_eq!(flow.segments().len(), 1);
        assert_eq!(
            flow.segments()
                .next()
                .unwrap()
                .picard()
                .evidence()
                .duration(),
            Some(&d(7, 7, -3))
        );
    }
    let QrWindowEvolution::Taylor(first_retained) =
        r.windows()[0].retained().unwrap().evolution().unwrap()
    else {
        panic!("Taylor strategy")
    };
    assert_eq!(first_retained.segments().len(), 2);
}
