#![cfg(feature = "field")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, FieldError,
    evolution::*,
    picard::{PicardFailure, PicardStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::BoxN;
use hyperreal::Rational;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};
const T: SymbolId = SymbolId(100);
const X: SymbolId = SymbolId(0);
const V: SymbolId = SymbolId(1);
fn d(a: i64, b: i64) -> DyadicInterval {
    scaled(a, b, 0)
}
fn scaled(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn x() -> Expr {
    Expr::anonymous_symbol(X)
}
fn definition<const N: usize>(outputs: &[Expr]) -> FieldDefinition {
    let states: Vec<_> = (0..N).map(|i| SymbolId(i as u32)).collect();
    let mut variables = states.clone();
    variables.reverse();
    variables.push(T);
    let source = ExpressionProgram::compile(
        outputs,
        &variables,
        &mut ExpressionBudget::new(100000, 10000000),
    )
    .unwrap();
    FieldDefinition {
        source: source.definition().clone(),
        time: T,
        state: states,
    }
}
fn field<const N: usize>(outputs: &[Expr]) -> ExpressionField<N> {
    ExpressionField::from_definition(definition::<N>(outputs), &mut b()).unwrap()
}
fn request<const N: usize>(
    end: DyadicInterval,
    initial: [DyadicInterval; N],
) -> EvolutionRequest<N> {
    EvolutionRequest {
        start: d(0, 0),
        end,
        initial: BoxN::new(initial).unwrap(),
    }
}
fn complete<const N: usize>(r: &EvolutionReport<N>) {
    assert!(
        matches!(r.status(), EvolutionStatus::Completed),
        "{:?}",
        r.status()
    );
    assert_eq!(r.reached_time(), &r.request().end);
    assert!(r.end_enclosure().is_some());
    let mut start = &r.request().start;
    let mut initial = &r.request().initial;
    for segment in r.segments() {
        assert!(matches!(segment.status(), PicardStatus::Certified));
        assert_eq!(&segment.proposal().start, start);
        assert_eq!(&segment.proposal().initial, initial);
        assert_eq!(segment.identity(), r.identity());
        assert_eq!(segment.policy(), r.policy());
        assert!(
            segment
                .proposal()
                .start
                .strictly_left_of(&segment.proposal().end, &mut scalar())
                .unwrap()
        );
        start = &segment.proposal().end;
        initial = segment.end_enclosure().unwrap();
    }
    for (i, a) in r.attempts().iter().enumerate() {
        match a.source() {
            CandidateSource::InitialBox => assert_eq!(
                a.report().proposal().candidate,
                a.report().proposal().initial
            ),
            CandidateSource::PicardExpansion { attempt } => {
                assert_eq!(attempt + 1, i);
                let old = &r.attempts()[attempt];
                assert_eq!(old.segment(), a.segment());
                assert_eq!(old.halvings(), a.halvings());
                assert_eq!(old.expansions() + 1, a.expansions());
                assert!(matches!(
                    old.report().status(),
                    PicardStatus::Rejected(PicardFailure::SelfMapNotCertified)
                ));
                let h = old
                    .report()
                    .proposal()
                    .candidate
                    .hull(
                        old.report().evidence().picard_image().unwrap(),
                        &mut scalar(),
                    )
                    .unwrap();
                assert_eq!(
                    a.report().proposal().candidate,
                    h.add(&h.sub(&h, &mut scalar()).unwrap(), &mut scalar())
                        .unwrap()
                );
            }
        }
        if i > 0
            && r.attempts()[i - 1].segment() == a.segment()
            && r.attempts()[i - 1].halvings() != a.halvings()
        {
            let old = &r.attempts()[i - 1];
            assert_eq!(old.halvings() + 1, a.halvings());
            assert_eq!(a.expansions(), 0);
            assert_eq!(
                a.report().proposal().end,
                old.report()
                    .proposal()
                    .start
                    .hull(&old.report().proposal().end, &mut scalar())
                    .unwrap()
                    .midpoint(&mut scalar())
                    .unwrap()
            );
        }
    }
}

#[test]
fn constant_uncertainty_exact_horizon_cap_and_endpoint_reuse() {
    for c in -3..=3 {
        for k in -3..=3 {
            let f = field::<1>(&[Expr::int(c)]);
            let options = EvolutionOptions {
                maximum_step: Some(scaled(1, 1, -2)),
                ..Default::default()
            };
            let r = f.integrate(
                request(scaled(5, 5, -3), [d(k, k + 1)]),
                options.clone(),
                &mut b(),
            );
            complete(&r);
            assert_eq!(r.segments().len(), 3);
            assert_eq!(
                r.end_enclosure().unwrap().lanes(),
                Some(&[scaled(8 * k + 5 * c, 8 * (k + 1) + 5 * c, -3)])
            );
            assert_eq!(r.attempts().len(), if c == 0 { 3 } else { 6 });
            assert_eq!(
                r.segments().last().unwrap().evidence().duration(),
                Some(&scaled(1, 1, -3))
            );
            let next = f.integrate(
                EvolutionRequest {
                    start: r.request().end.clone(),
                    end: d(1, 1),
                    initial: r.end_enclosure().unwrap().clone(),
                },
                options,
                &mut b(),
            );
            complete(&next);
            assert_eq!(
                next.end_enclosure().unwrap().lanes(),
                Some(&[d(k + c, k + 1 + c)])
            );
        }
    }
}

fn rational_point(d: &DyadicInterval) -> Rational {
    let r = d.to_rational(&mut scalar()).unwrap();
    assert!(r.is_singleton());
    r.lower().unwrap().clone()
}
fn enclosure(r: &Rational) -> DyadicInterval {
    DyadicInterval::from_rational(r, -80, &mut scalar()).unwrap()
}
fn included(value: &DyadicInterval, container: &DyadicInterval) {
    assert!(
        value.subset_of(container, &mut scalar()).unwrap(),
        "{value:?} not in {container:?}"
    );
}

#[test]
fn time_affine_oscillator_and_riccati_have_independent_analytic_controls() {
    let f = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T) + Expr::int(3)]);
    let r = f.integrate(
        request(scaled(3, 3, -2), [d(0, 0)]),
        EvolutionOptions {
            maximum_step: Some(scaled(1, 1, -2)),
            ..Default::default()
        },
        &mut b(),
    );
    complete(&r);
    for s in r.segments() {
        let t = rational_point(&s.proposal().end);
        let exact = &t * &t + Rational::from(3) * t;
        included(
            &enclosure(&exact),
            &s.end_enclosure().unwrap().lanes().unwrap()[0],
        );
    }
    // x'=2x at x0=0 has the exact zero flow, but h=1/2 gives q=1.
    // This forces halving independently of candidate-growth heuristics.
    let linear = field::<1>(&[Expr::int(2) * x()]);
    let zero = linear.integrate(
        request(scaled(1, 1, -1), [d(0, 0)]),
        EvolutionOptions::default(),
        &mut b(),
    );
    complete(&zero);
    assert!(zero.attempts().iter().any(|a| a.halvings() > 0));
    assert_eq!(zero.attempts().len(), 3);
    assert!(matches!(
        zero.attempts()[0].report().status(),
        PicardStatus::Rejected(PicardFailure::ContractionNotCertified)
    ));
    assert_eq!(zero.segments().len(), 2);
    for segment in zero.segments() {
        assert_eq!(segment.end_enclosure().unwrap().lanes(), Some(&[d(0, 0)]));
        assert_eq!(segment.evidence().duration(), Some(&scaled(1, 1, -2)));
    }
    let exponential = field::<1>(&[x()]);
    let exp_flow = exponential.integrate(
        request(scaled(1, 1, -2), [d(1, 1)]),
        EvolutionOptions::default(),
        &mut b(),
    );
    complete(&exp_flow);
    // exp(t): positive Taylor sum through degree eight and a geometric tail
    // bound, since successive tail ratios are at most t/10 on this horizon.
    let exp_bounds = |time: &DyadicInterval| {
        let t = rational_point(time);
        let mut term = Rational::from(1);
        let mut sum = term.clone();
        for k in 1..=8 {
            term = &term * &t / Rational::from(k);
            sum = &sum + &term;
        }
        let tail = term * &t / Rational::from(9) / (Rational::from(1) - t / Rational::from(10));
        enclosure(&sum)
            .hull(&enclosure(&(sum + tail)), &mut scalar())
            .unwrap()
    };
    for segment in exp_flow.segments() {
        let end = exp_bounds(&segment.proposal().end);
        included(&end, &segment.end_enclosure().unwrap().lanes().unwrap()[0]);
        included(
            &exp_bounds(&segment.proposal().start)
                .hull(&end, &mut scalar())
                .unwrap(),
            &segment.tube().unwrap().lanes().unwrap()[0],
        );
    }
    let oscillator = field::<2>(&[Expr::anonymous_symbol(V), -x()]);
    for options in [
        EvolutionOptions::default(),
        EvolutionOptions {
            maximum_step: Some(scaled(1, 1, -3)),
            ..Default::default()
        },
    ] {
        let r = oscillator.integrate(
            request(scaled(1, 1, -1), [d(1, 1), d(0, 0)]),
            options,
            &mut b(),
        );
        complete(&r);

        // Alternating Taylor bounds on [0,1/2], with exact rational remainders.
        for s in r.segments() {
            let t = rational_point(&s.proposal().end);
            let t2 = &t * &t;
            let t3 = &t2 * &t;
            let t4 = &t2 * &t2;
            let cos_lo = Rational::from(1) - &t2 / Rational::from(2);
            let cos_hi = &cos_lo + &t4 / Rational::from(24);
            let sin_lo = &t - &t3 / Rational::from(6);
            let ex = enclosure(&cos_lo)
                .hull(&enclosure(&cos_hi), &mut scalar())
                .unwrap();
            let ey = enclosure(&(-&t))
                .hull(&enclosure(&(-sin_lo)), &mut scalar())
                .unwrap();
            let end = s.end_enclosure().unwrap().lanes().unwrap();
            included(&ex, &end[0]);
            included(&ey, &end[1]);
            let a = rational_point(&s.proposal().start);
            let a2 = &a * &a;
            let a3 = &a2 * &a;
            let tx = enclosure(&cos_lo)
                .hull(&enclosure(&Rational::from(1)), &mut scalar())
                .unwrap();
            let ty = enclosure(&(-t))
                .hull(&enclosure(&(-a + a3 / Rational::from(6))), &mut scalar())
                .unwrap();
            let tube = s.tube().unwrap().lanes().unwrap();
            // A stronger global upper bound remains within these ordinary proposals.
            included(&tx, &tube[0]);
            included(&ty, &tube[1]);
        }
    }
    let riccati = field::<1>(&[x().powi(2)]);
    let r = riccati.integrate(
        request(scaled(1, 1, -2), [d(1, 1)]),
        EvolutionOptions::default(),
        &mut b(),
    );
    complete(&r);
    for s in r.segments() {
        let at =
            |time: &DyadicInterval| Rational::from(1) / (Rational::from(1) - rational_point(time));
        let lo = enclosure(&at(&s.proposal().start));
        let hi = enclosure(&at(&s.proposal().end));
        included(&hi, &s.end_enclosure().unwrap().lanes().unwrap()[0]);
        included(
            &lo.hull(&hi, &mut scalar()).unwrap(),
            &s.tube().unwrap().lanes().unwrap()[0],
        );
    }
}

#[test]
fn stopped_horizons_retain_certified_prefix_and_every_rejection() {
    let f = field::<1>(&[Expr::int(1)]);
    let req = request(scaled(5, 5, -3), [d(0, 0)]);
    let r = f.integrate(
        req.clone(),
        EvolutionOptions {
            maximum_step: Some(scaled(1, 1, -2)),
            maximum_steps: 1,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        r.status(),
        EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 })
    ));
    assert_eq!(r.segments().len(), 1);
    assert_eq!(r.reached_time(), &scaled(1, 1, -2));
    assert!(r.end_enclosure().is_none());
    assert_eq!(
        r.prefix_end_enclosure().unwrap().lanes(),
        Some(&[scaled(1, 1, -2)])
    );
    let r = f.integrate(
        req,
        EvolutionOptions {
            maximum_expansions: 0,
            maximum_halvings: 2,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        r.status(),
        EvolutionStatus::Stopped(EvolutionFailure::HalvingLimit { limit: 2 })
    ));
    assert_eq!(r.attempts().len(), 3);
    assert_eq!(r.segments().len(), 0);
    assert!(r.end_enclosure().is_none());
    for (i, a) in r.attempts().iter().enumerate() {
        assert_eq!(a.halvings(), i as u32);
        assert_eq!(a.report().proposal().end, scaled(5, 5, -3 - i as i32));
        assert!(matches!(
            a.report().status(),
            PicardStatus::Rejected(PicardFailure::SelfMapNotCertified)
        ));
    }
    let pole = field::<1>(&[Expr::int(1) / x()]);
    let r = pole.integrate(
        request(d(1, 1), [d(0, 0)]),
        EvolutionOptions {
            maximum_halvings: 2,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        r.status(),
        EvolutionStatus::Stopped(EvolutionFailure::HalvingLimit { limit: 2 })
    ));
    assert_eq!(r.attempts().len(), 3);
    assert!(r.attempts().iter().all(|a| matches!(
        a.report().status(),
        PicardStatus::Rejected(PicardFailure::Field(_))
    )));
    let blowup = field::<1>(&[x().powi(2)]).integrate(
        request(d(1, 1), [d(1, 1)]),
        EvolutionOptions {
            maximum_steps: 24,
            maximum_halvings: 8,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(blowup.status(), EvolutionStatus::Stopped(_)));
    assert!(blowup.end_enclosure().is_none());
    assert!(
        blowup
            .reached_time()
            .strictly_left_of(&d(1, 1), &mut scalar())
            .unwrap()
    );
}

#[test]
fn zero_horizon_empty_unit_and_invalid_inputs_are_distinct() {
    let f = field::<0>(&[]);
    let req = request(d(1, 1), []);
    let unit = f.integrate(req.clone(), Default::default(), &mut b());
    complete(&unit);
    assert!(!unit.end_enclosure().unwrap().is_empty());
    let mut p = req.clone();
    p.initial = BoxN::empty().unwrap();
    let empty = f.integrate(
        p.clone(),
        EvolutionOptions {
            maximum_steps: 0,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(empty.status(), EvolutionStatus::Empty));
    assert!(empty.end_enclosure().unwrap().is_empty());
    assert_eq!(empty.reached_time(), &empty.request().end);
    assert!(empty.attempts().is_empty());
    p.start = DyadicInterval::empty();
    assert!(matches!(
        f.integrate(p, Default::default(), &mut b()).status(),
        EvolutionStatus::Stopped(EvolutionFailure::NonPointTime)
    ));
    let mut p = req.clone();
    p.end = d(-1, -1);
    assert!(matches!(
        f.integrate(p, Default::default(), &mut b()).status(),
        EvolutionStatus::Stopped(EvolutionFailure::ReversedTime)
    ));
    for maximum_step in [DyadicInterval::empty(), d(0, 0), d(-1, -1), d(1, 2)] {
        assert!(matches!(
            f.integrate(
                req.clone(),
                EvolutionOptions {
                    maximum_step: Some(maximum_step),
                    ..Default::default()
                },
                &mut b()
            )
            .status(),
            EvolutionStatus::Stopped(EvolutionFailure::InvalidMaximumStep)
        ));
    }
    let mut p = req;
    p.end = d(0, 0);
    let identity = f.integrate(
        p,
        EvolutionOptions {
            maximum_steps: 0,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(identity.status(), EvolutionStatus::Identity));
    assert!(!identity.end_enclosure().unwrap().is_empty());
    assert!(identity.attempts().is_empty());
}

#[test]
fn aggregate_work_limits_final_check_cancellation_and_retry() {
    let f = field::<1>(&[Expr::int(1)]);
    let req = request(scaled(5, 5, -3), [d(0, 0)]);
    let options = EvolutionOptions {
        maximum_step: Some(scaled(1, 1, -2)),
        ..Default::default()
    };
    let mut measured = b();
    complete(&f.integrate(req.clone(), options.clone(), &mut measured));
    let p = measured.policy();
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw,
        scalar_work: sw,
        ..p
    });
    complete(&f.integrate(req.clone(), options.clone(), &mut exact));
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    let last = f.integrate(
        req.clone(),
        options.clone(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            scalar_work: sw - 1,
            ..p
        }),
    );
    assert!(matches!(
        last.status(),
        EvolutionStatus::Stopped(EvolutionFailure::Field(FieldError::Scalar(
            IntervalError::WorkExhausted
        )))
    ));
    assert_eq!(last.segments().len(), 3);
    assert_eq!(last.reached_time(), &req.end);
    assert!(last.prefix_end_enclosure().is_some());
    assert!(last.end_enclosure().is_none());
    let graph = f.integrate(
        req.clone(),
        options.clone(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            graph_work: gw - 1,
            ..p
        }),
    );
    assert!(matches!(graph.status(), EvolutionStatus::Stopped(_)));
    assert!(graph.end_enclosure().is_none());
    let mut two = EnclosureBudget::new(EnclosurePolicy {
        graph_work: 2 * gw,
        scalar_work: 2 * sw,
        ..p
    });
    complete(&f.integrate(req.clone(), options.clone(), &mut two));
    complete(&f.integrate(req.clone(), options.clone(), &mut two));
    assert_eq!(two.remaining_graph_work(), 0);
    assert_eq!(two.remaining_scalar_work(), 0);
    let token = CancellationToken::new();
    token.cancel();
    let mut work = b();
    work.set_cancellation(Some(token));
    let cancelled = f.integrate(req.clone(), options.clone(), &mut work);
    assert!(matches!(
        cancelled.status(),
        EvolutionStatus::Stopped(EvolutionFailure::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert!(cancelled.attempts().is_empty());
    let small = f.integrate(
        req.clone(),
        options.clone(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            graph_entries: 1,
            ..p
        }),
    );
    assert_eq!(small.attempts().len(), 1);
    assert!(matches!(
        small.status(),
        EvolutionStatus::Stopped(EvolutionFailure::Step(PicardFailure::Field(
            FieldError::Expression(ExpressionError::SizeExhausted)
        )))
    ));
    complete(&f.integrate(req, options, &mut b()));
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_all_attempts_and_rejects_changed_requests() {
    fn signature(r: &EvolutionReport<1>) -> String {
        complete(r);
        let attempts: Vec<_> = r
            .attempts()
            .iter()
            .map(|a| {
                let p = a.report();
                let e = p.evidence();
                serde_json::json!([
                    a.segment(),
                    a.halvings(),
                    a.expansions(),
                    format!("{:?}", a.source()),
                    p.identity(),
                    p.proposal(),
                    p.policy(),
                    e.time(),
                    e.duration(),
                    e.image(),
                    e.jacobian(),
                    e.picard_image(),
                    e.lipschitz(),
                    e.contraction(),
                    e.end(),
                    format!("{:?}", p.status())
                ])
            })
            .collect();
        serde_json::to_string(&(
            r.identity(),
            r.request(),
            r.options(),
            r.policy(),
            attempts,
            r.reached_time(),
            r.end_enclosure(),
            format!("{:?}", r.status()),
        ))
        .unwrap()
    }
    const KEY: &str = "HYPERDYNAMICS_EVOLUTION_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (source, request, options, policy): (
            FieldDefinition,
            EvolutionRequest<1>,
            EvolutionOptions,
            EnclosurePolicy,
        ) = serde_json::from_str(&encoded).unwrap();
        let f = ExpressionField::<1>::from_definition(source, &mut b()).unwrap();
        println!(
            "EVOLUTION_REPLAY={}",
            signature(&f.integrate(request, options, &mut EnclosureBudget::new(policy)))
        );
        return;
    }
    let source = definition::<1>(&[x().powi(2)]);
    let f = ExpressionField::<1>::from_definition(source, &mut b()).unwrap();
    let req = request(scaled(1, 1, -2), [d(1, 1)]);
    let options = EvolutionOptions::default();
    let original = f.integrate(req.clone(), options.clone(), &mut b());
    let expected = signature(&original);
    let encoded = serde_json::to_string(&(
        original.identity(),
        &req,
        &options,
        EnclosurePolicy::default(),
    ))
    .unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_rebuilds_all_attempts_and_rejects_changed_requests",
            "--nocapture",
        ])
        .env(KEY, encoded)
        .output()
        .unwrap();
    assert!(
        child.status.success(),
        "{}",
        String::from_utf8_lossy(&child.stderr)
    );
    let stdout = String::from_utf8(child.stdout).unwrap();
    assert_eq!(
        stdout
            .lines()
            .find_map(|s| s.strip_prefix("EVOLUTION_REPLAY="))
            .unwrap(),
        expected
    );
    let mut changed = req;
    changed.end = d(-1, -1);
    assert!(matches!(
        f.integrate(changed, options, &mut b()).status(),
        EvolutionStatus::Stopped(EvolutionFailure::ReversedTime)
    ));
    assert_eq!(signature(&original), expected);
}

#[test]
fn negative_start_time_and_unrepresentable_halving_keep_exact_semantics() {
    let f = field::<1>(&[Expr::int(1)]);
    let r = f.integrate(
        EvolutionRequest {
            start: scaled(-3, -3, -3),
            end: scaled(1, 1, -2),
            initial: BoxN::new([d(0, 0)]).unwrap(),
        },
        EvolutionOptions {
            maximum_step: Some(scaled(1, 1, -2)),
            ..Default::default()
        },
        &mut b(),
    );
    complete(&r);
    assert_eq!(
        r.end_enclosure().unwrap().lanes(),
        Some(&[scaled(5, 5, -3)])
    );
    let zero = f.integrate(
        request(d(1, 1), [d(0, 0)]),
        EvolutionOptions {
            maximum_steps: 0,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        zero.status(),
        EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 0 })
    ));
    assert!(zero.attempts().is_empty());
    let underflow = f.integrate(
        request(scaled(1, 1, i32::MIN), [d(0, 0)]),
        EvolutionOptions {
            maximum_expansions: 0,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(
        matches!(
            underflow.status(),
            EvolutionStatus::Stopped(EvolutionFailure::Field(FieldError::Scalar(
                IntervalError::Overflow
            )))
        ),
        "{:?}",
        underflow.status()
    );
    assert_eq!(underflow.attempts().len(), 1);
    assert!(underflow.end_enclosure().is_none());
}
