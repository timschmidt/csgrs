#![cfg(feature = "adaptive-taylor")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionFailure, EvolutionOptions, EvolutionRequest},
    taylor_flow::{
        FlowTaylorStatus,
        adaptive::{
            AdaptiveTaylorFailure as Failure, AdaptiveTaylorOptions, AdaptiveTaylorStatus as Status,
        },
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::Rational as Q;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

#[path = "support/exponential_oracle.rs"]
mod exponential_oracle;
use exponential_oracle::exp_bounds;

const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000)
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
fn request<const N: usize>(initial: [DyadicInterval; N]) -> EvolutionRequest<N> {
    EvolutionRequest {
        start: d(0, 0, 0),
        end: d(1, 1, -1),
        initial: BoxN::new(initial).unwrap(),
    }
}
fn options<const N: usize>() -> AdaptiveTaylorOptions<N> {
    AdaptiveTaylorOptions {
        maximum_order: 3,
        remainder_limits: std::array::from_fn(|_| d(1, 1, -10)),
        native: EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        },
        ..Default::default()
    }
}
#[test]
fn constant_flow_escalates_past_nonzero_singleton_remainder_and_shares_orders() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let mut o = options();
    o.remainder_limits = [d(0, 0, 0)];
    let c = f.adaptive_taylor(o);
    let r = c.integrate(request([d(0, 0, 0)]), &mut budget());
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    assert_eq!(r.end_enclosure(), Some(&BoxN::new([d(1, 1, 0)]).unwrap()));
    assert_eq!(
        r.programs().iter().map(|p| p.order()).collect::<Vec<_>>(),
        [0, 1]
    );
    assert_eq!(r.segments().len(), 2);
    for a in r.attempts() {
        assert_eq!(a.refinements().len(), 2);
        let rejected = &a.refinements()[0];
        assert_eq!(rejected.meets_limits(), Some(false));
        assert!(matches!(rejected.status(), FlowTaylorStatus::Certified));
        assert_eq!(rejected.remainder_magnitudes().unwrap(), &[d(1, 1, -1)]);
        assert_eq!(a.refinements()[1].meets_limits(), Some(true));
        assert_eq!(
            a.refinements()[1].remainder_magnitudes().unwrap(),
            &[d(0, 0, 0)]
        );
    }
    for s in r.segments() {
        assert_eq!(s.program().order(), 1);
        assert_eq!(s.program().identity(), f.identity());
    }
    let segments = r.segments().collect::<Vec<_>>();
    assert_eq!(
        &segments[1].picard().proposal().initial,
        segments[0].end_enclosure()
    );
    let continuation = c.integrate(
        EvolutionRequest {
            start: r.reached_time().clone(),
            end: d(1, 1, 0),
            initial: r.end_enclosure().unwrap().clone(),
        },
        &mut budget(),
    );
    assert_eq!(
        continuation.end_enclosure(),
        Some(&BoxN::new([d(2, 2, 0)]).unwrap())
    );
}
#[test]
fn quality_halves_at_last_order_then_regrows_and_caps_exact_horizon() {
    let f = field::<1>(&[Expr::int(1) - Expr::anonymous_symbol(T)], &[X]);
    let c = f.adaptive_taylor(AdaptiveTaylorOptions {
        maximum_order: 0,
        remainder_limits: [d(1, 1, -2)],
        native: EvolutionOptions {
            maximum_step: None,
            ..Default::default()
        },
        ..Default::default()
    });
    let r = c.integrate(
        EvolutionRequest {
            end: d(1, 1, 0),
            ..request([d(0, 0, 0)])
        },
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    let durations = r
        .segments()
        .map(|s| s.picard().evidence().duration().unwrap().clone())
        .collect::<Vec<_>>();
    assert_eq!(durations, [d(1, 1, -2), d(1, 1, -2), d(1, 1, -1)]);
    assert_eq!(r.reached_time(), &d(1, 1, 0));
    assert_eq!(r.programs().len(), 1);
    assert_eq!(
        r.attempts()
            .iter()
            .map(|a| a.quality_halvings())
            .collect::<Vec<_>>(),
        [0, 1, 2, 0, 1, 0]
    );
    assert!(
        d(1, 1, -1)
            .subset_of(
                &r.end_enclosure().unwrap().lanes().unwrap()[0],
                &mut arithmetic()
            )
            .unwrap()
    );
    assert!(
        r.attempts()
            .iter()
            .flat_map(|a| a.refinements())
            .any(|r| r.meets_limits() == Some(false))
    );
    for s in r.segments() {
        assert_eq!(s.refinement().meets_limits(), Some(true));
        assert!(
            s.refinement().remainder_magnitudes().unwrap()[0]
                .subset_of(&d(0, 1, -2), &mut arithmetic())
                .unwrap()
        );
    }
}
#[test]
fn exponential_uncertainty_and_generated_endpoint_have_independent_bounds() {
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    let c = f.adaptive_taylor(options());
    let r = c.integrate(request([d(1, 2, 0)]), &mut budget());
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    let end = r.end_enclosure().unwrap().lanes().unwrap()[0]
        .to_rational(&mut arithmetic())
        .unwrap();
    let (lo, hi) = exp_bounds(Q::from(1) / Q::from(2));
    assert!(end.contains_point(&lo));
    assert!(end.contains_point(&(hi * Q::from(2))));
    assert!(r.programs().iter().any(|p| p.order() == 3));
    for s in r.segments() {
        assert_eq!(s.refinement().meets_limits(), Some(true));
    }
    let continued = c.integrate(
        EvolutionRequest {
            start: r.reached_time().clone(),
            end: d(3, 3, -2),
            initial: r.end_enclosure().unwrap().clone(),
        },
        &mut budget(),
    );
    assert!(
        matches!(continued.status(), Status::Completed),
        "{:?}",
        continued.status()
    );
    let end = continued.end_enclosure().unwrap().lanes().unwrap()[0]
        .to_rational(&mut arithmetic())
        .unwrap();
    let (lo, hi) = exp_bounds(Q::from(3) / Q::from(4));
    assert!(end.contains_point(&lo));
    assert!(end.contains_point(&(hi * Q::from(2))));
}
#[test]
fn oscillator_keeps_signed_remainders_and_independent_alternating_bounds() {
    let v = SymbolId(30);
    let f = field::<2>(
        &[Expr::anonymous_symbol(v), -Expr::anonymous_symbol(X)],
        &[X, v],
    );
    let c = f.adaptive_taylor(options());
    let r = c.integrate(
        EvolutionRequest {
            end: d(1, 1, -2),
            ..request([d(1, 1, 0), d(0, 0, 0)])
        },
        &mut budget(),
    );
    assert!(matches!(r.status(), Status::Completed), "{:?}", r.status());
    let t = Q::from(1) / Q::from(4);
    let t2 = &t * &t;
    let mut cosine = Q::from(1);
    let mut sine = t.clone();
    let mut ct = Q::from(1);
    let mut st = t;
    for k in 1..=6 {
        ct = -ct * &t2 / Q::from((2 * k - 1) * (2 * k));
        st = -st * &t2 / Q::from((2 * k) * (2 * k + 1));
        cosine = cosine + &ct;
        sine = sine + &st;
    }
    let cos_next = -ct * &t2 / Q::from(13 * 14);
    let sin_next = -st * &t2 / Q::from(14 * 15);
    let lanes = r.end_enclosure().unwrap().lanes().unwrap();
    let cx = lanes[0].to_rational(&mut arithmetic()).unwrap();
    let sv = lanes[1].to_rational(&mut arithmetic()).unwrap();
    assert!(cx.contains_point(&cosine));
    assert!(cx.contains_point(&(&cosine + cos_next)));
    assert!(sv.contains_point(&(-sine.clone())));
    assert!(sv.contains_point(&(-(&sine + sin_next))));
}
#[test]
fn invalid_inputs_precede_empty_identity_and_unit_success() {
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let empty = EvolutionRequest {
        initial: BoxN::empty().unwrap(),
        ..request([d(0, 0, 0)])
    };
    let r = f
        .adaptive_taylor(options())
        .integrate(empty.clone(), &mut budget());
    assert!(matches!(r.status(), Status::Empty));
    assert!(r.end_enclosure().unwrap().is_empty());
    assert_eq!(r.reached_time(), &empty.end);
    assert!(r.programs().is_empty());
    for o in [
        AdaptiveTaylorOptions {
            minimum_order: 4,
            maximum_order: 3,
            ..options()
        },
        AdaptiveTaylorOptions {
            maximum_order: usize::MAX,
            ..options()
        },
    ] {
        let r = f.adaptive_taylor(o).integrate(empty.clone(), &mut budget());
        assert!(matches!(
            r.status(),
            Status::Stopped(Failure::InvalidOrderRange)
        ));
        assert!(r.end_enclosure().is_none());
    }
    for limit in [d(-1, -1, 0), d(0, 1, 0), DyadicInterval::empty()] {
        let r = f
            .adaptive_taylor(AdaptiveTaylorOptions {
                remainder_limits: [limit],
                ..options()
            })
            .integrate(empty.clone(), &mut budget());
        assert!(matches!(
            r.status(),
            Status::Stopped(Failure::InvalidRemainderLimit { coordinate: 0 })
        ));
        assert!(r.end_enclosure().is_none());
    }
    let c = f.adaptive_taylor(options());
    let r = c.integrate(
        EvolutionRequest {
            start: d(2, 2, 0),
            ..empty
        },
        &mut budget(),
    );
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Search(EvolutionFailure::ReversedTime))
    ));
    let req = EvolutionRequest {
        end: d(0, 0, 0),
        ..request([d(1, 2, 0)])
    };
    let r = c.integrate(req.clone(), &mut budget());
    assert!(matches!(r.status(), Status::Identity));
    assert_eq!(r.end_enclosure(), Some(&req.initial));
    assert!(r.programs().is_empty());
    let unit = field::<0>(&[], &[])
        .adaptive_taylor(options())
        .integrate(request([]), &mut budget());
    assert!(
        matches!(unit.status(), Status::Completed),
        "{:?}",
        unit.status()
    );
    assert!(!unit.end_enclosure().unwrap().is_empty());
}
#[test]
fn quality_and_native_limits_are_distinct_and_prefix_is_never_whole_success() {
    let f = field::<1>(&[Expr::int(-2)], &[X]);
    let c = f.adaptive_taylor(AdaptiveTaylorOptions {
        maximum_order: 0,
        remainder_limits: [d(0, 0, 0)],
        maximum_quality_halvings: 0,
        ..options()
    });
    let r = c.integrate(request([d(0, 0, 0)]), &mut budget());
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::QualityHalvingLimit { limit: 0 })
    ));
    assert!(r.end_enclosure().is_none());
    assert!(r.prefix_end_enclosure().is_none());
    assert_eq!(r.attempts()[0].refinements()[0].meets_limits(), Some(false));
    assert_eq!(
        r.attempts()[0].refinements()[0]
            .remainder_magnitudes()
            .unwrap(),
        &[d(1, 1, -1)]
    );
    let c = f.adaptive_taylor(AdaptiveTaylorOptions {
        native: EvolutionOptions {
            maximum_steps: 1,
            ..options::<1>().native
        },
        ..options()
    });
    let r = c.integrate(request([d(0, 0, 0)]), &mut budget());
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::Search(EvolutionFailure::StepLimit { limit: 1 }))
    ));
    assert!(r.end_enclosure().is_none());
    assert_eq!(
        r.prefix_end_enclosure(),
        Some(&BoxN::new([d(-1, -1, -1)]).unwrap())
    );
    let f = field::<1>(&[Expr::int(1) / Expr::anonymous_symbol(X)], &[X]);
    let r = f
        .adaptive_taylor(options())
        .integrate(request([d(-1, 1, 0)]), &mut budget());
    assert!(matches!(r.status(), Status::Stopped(Failure::Search(_))));
    assert!(r.end_enclosure().is_none());
    assert!(r.programs().is_empty());
}
#[test]
fn exact_final_budgets_all_graph_cutoffs_cancellation_and_retry() {
    let c = field::<1>(&[Expr::int(2)], &[X]).adaptive_taylor(options());
    let req = EvolutionRequest {
        end: d(1, 1, -2),
        ..request([d(0, 0, 0)])
    };
    let mut full_budget = budget();
    let full = c.integrate(req.clone(), &mut full_budget);
    assert!(matches!(full.status(), Status::Completed));
    let graph = EnclosurePolicy::default().graph_work - full_budget.remaining_graph_work();
    let scalar = EnclosurePolicy::default().scalar_work - full_budget.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: graph,
        scalar_work: scalar,
        ..Default::default()
    });
    let r = c.integrate(req.clone(), &mut exact);
    assert!(matches!(r.status(), Status::Completed));
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    for work in 0..graph {
        let r = c.integrate(
            req.clone(),
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_work: work,
                ..Default::default()
            }),
        );
        assert!(
            matches!(r.status(), Status::Stopped(_)),
            "cutoff {work}: {:?}",
            r.status()
        );
        assert!(r.end_enclosure().is_none());
    }
    for policy in [
        EnclosurePolicy {
            scalar_work: scalar - 1,
            ..Default::default()
        },
        EnclosurePolicy {
            graph_entries: 0,
            ..Default::default()
        },
    ] {
        let r = c.integrate(req.clone(), &mut EnclosureBudget::new(policy));
        assert!(matches!(r.status(), Status::Stopped(_)));
        assert!(r.end_enclosure().is_none());
    }
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token));
    let r = c.integrate(req.clone(), &mut cancelled);
    assert!(matches!(r.status(), Status::Stopped(_)));
    assert!(r.attempts().is_empty());
    assert!(r.end_enclosure().is_none());
    let retry = c.integrate(req, &mut budget());
    assert_eq!(retry.end_enclosure(), full.end_enclosure());
    assert_eq!(retry.attempts().len(), full.attempts().len());
}
#[test]
fn analytic_failure_is_not_retried_as_quality_or_replaced_by_picard() {
    use hyperdynamics::field::taylor_flow::FlowTaylorError;
    let f = field::<1>(&[Expr::anonymous_symbol(T).sqrt()], &[X]);
    let r = f
        .adaptive_taylor(options())
        .integrate(request([d(0, 0, 0)]), &mut budget());
    assert!(
        matches!(
            r.status(),
            Status::Stopped(Failure::Taylor(FlowTaylorError::Analytic(_)))
        ),
        "{:?}",
        r.status()
    );
    assert_eq!(r.attempts().len(), 1);
    assert_eq!(r.attempts()[0].search().segments().len(), 1);
    assert_eq!(r.attempts()[0].refinements().len(), 1);
    assert_eq!(r.attempts()[0].refinements()[0].meets_limits(), None);
    assert!(r.end_enclosure().is_none());
    assert!(r.prefix_end_enclosure().is_none());
    let v = SymbolId(30);
    let f = field::<2>(&[Expr::int(0), Expr::int(2)], &[X, v]);
    let r = f
        .adaptive_taylor(AdaptiveTaylorOptions {
            maximum_order: 0,
            remainder_limits: [d(0, 0, 0), d(0, 0, 0)],
            maximum_quality_halvings: 0,
            ..options()
        })
        .integrate(request([d(0, 0, 0), d(0, 0, 0)]), &mut budget());
    assert!(matches!(
        r.status(),
        Status::Stopped(Failure::QualityHalvingLimit { limit: 0 })
    ));
    assert_eq!(
        r.attempts()[0].refinements()[0]
            .remainder_magnitudes()
            .unwrap(),
        &[d(0, 0, 0), d(1, 1, -1)]
    );
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_source_request_and_selection_replay() {
    const KEY: &str = "HYPERDYNAMICS_ADAPTIVE_TAYLOR_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (
            definition,
            req,
            minimum_order,
            maximum_order,
            limits,
            maximum_quality_halvings,
            native,
        ): (
            FieldDefinition,
            EvolutionRequest<1>,
            usize,
            usize,
            Vec<DyadicInterval>,
            u32,
            EvolutionOptions,
        ) = serde_json::from_str(&data).unwrap();
        let mut b = budget();
        let f = ExpressionField::<1>::from_definition(definition, &mut b).unwrap();
        let r = f
            .adaptive_taylor(AdaptiveTaylorOptions {
                minimum_order,
                maximum_order,
                remainder_limits: limits.try_into().unwrap(),
                maximum_quality_halvings,
                native,
            })
            .integrate(req, &mut b);
        assert!(matches!(r.status(), Status::Completed));
        let (lo, hi) = exp_bounds(Q::from(1) / Q::from(2));
        let end = r.end_enclosure().unwrap().lanes().unwrap()[0]
            .to_rational(&mut arithmetic())
            .unwrap();
        assert!(end.contains_point(&lo));
        assert!(end.contains_point(&hi));
        println!(
            "REPLAY {:?} GRAPH {} SCALAR {}",
            r,
            b.remaining_graph_work(),
            b.remaining_scalar_work()
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
    let o = options::<1>();
    let data = serde_json::to_string(&(
        definition,
        request([d(1, 1, 0)]),
        o.minimum_order,
        o.maximum_order,
        o.remainder_limits.to_vec(),
        o.maximum_quality_halvings,
        o.native,
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "fresh_process_source_request_and_selection_replay",
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
#[cfg(feature = "dispatch-trace")]
#[test]
fn all_retained_evidence_getters_do_not_repeat_decisions() {
    use hyperreal::dispatch_trace;
    let c = field::<1>(&[Expr::int(2)], &[X]).adaptive_taylor(options());
    let r = c.integrate(request([d(0, 0, 0)]), &mut budget());
    assert!(matches!(r.status(), Status::Completed));
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        let _ = (
            r.controller(),
            r.controller().field(),
            r.controller().options(),
            r.request(),
            r.policy(),
            r.programs(),
            r.attempts(),
            r.status(),
            r.reached_time(),
            r.prefix_end_enclosure(),
            r.end_enclosure(),
        );
        for a in r.attempts() {
            let _ = (a.quality_halvings(), a.search(), a.refinements());
            for f in a.refinements() {
                let _ = (
                    f.program_index(),
                    f.evidence(),
                    f.status(),
                    f.remainder_magnitudes(),
                    f.meets_limits(),
                );
            }
        }
        for s in r.segments() {
            let _ = (
                s.program(),
                s.picard(),
                s.refinement(),
                s.tube(),
                s.end_enclosure(),
            );
        }
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
