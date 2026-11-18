#![cfg(feature = "taylor-flow")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    picard::PicardProposal,
    taylor_flow::{FlowTaylorStatus, orders::FlowTaylorOrderError},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
};
const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn field<const N: usize>(outputs: &[Expr], state: &[SymbolId]) -> ExpressionField<N> {
    let bindings: Vec<_> = state.iter().rev().copied().chain([T]).collect();
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
fn step<const N: usize>(
    initial: [DyadicInterval; N],
    candidate: [DyadicInterval; N],
) -> PicardProposal<N> {
    PicardProposal {
        start: d(0, 0, 0),
        end: d(1, 1, -3),
        initial: BoxN::new(initial).unwrap(),
        candidate: BoxN::new(candidate).unwrap(),
    }
}
fn compare<const N: usize>(f: &ExpressionField<N>, proposal: PicardProposal<N>) {
    let high = f.flow_taylor(4, &mut budget()).unwrap();
    for order in 0..=4 {
        let prefix = high.restrict_order(order, &mut budget()).unwrap();
        let fresh = f.flow_taylor(order, &mut budget()).unwrap();
        assert_eq!(prefix.order(), order);
        assert_eq!(prefix.stages().len(), order + 1);
        assert_eq!(prefix.identity(), fresh.identity());
        for ((a, b), shared) in prefix
            .stages()
            .iter()
            .zip(fresh.stages())
            .zip(high.stages())
        {
            assert_eq!(a.degree(), b.degree());
            assert_eq!(a.outputs(), b.outputs());
            assert_eq!(a.program().definition(), b.program().definition());
            assert!(std::ptr::eq(
                a.program().definition(),
                shared.program().definition()
            ));
            match (a.derivatives(), b.derivatives(), shared.derivatives()) {
                (Some(a), Some(b), Some(c)) => {
                    assert_eq!(a.requests(), b.requests());
                    assert_eq!(a.source().definition(), b.source().definition());
                    assert_eq!(a.program().definition(), b.program().definition());
                    assert!(std::ptr::eq(a.requests().as_ptr(), c.requests().as_ptr()));
                    assert!(std::ptr::eq(
                        a.program().definition(),
                        c.program().definition()
                    ));
                }
                (None, None, None) => {}
                _ => panic!("derivative association changed"),
            }
        }
        let mut a = budget();
        let mut b = budget();
        let actual = prefix.certify_step(proposal.clone(), &mut a);
        let expected = fresh.certify_step(proposal.clone(), &mut b);
        assert!(
            matches!(actual.status(), FlowTaylorStatus::Certified),
            "{:?}",
            actual.status()
        );
        // Full deterministic proof artifacts, not just endpoint boxes.
        assert_eq!(format!("{actual:?}"), format!("{expected:?}"));
        assert_eq!(a.remaining_graph_work(), b.remaining_graph_work());
        assert_eq!(a.remaining_scalar_work(), b.remaining_scalar_work());
        assert_eq!(actual.evidence().coefficients().len(), order);
        assert!(actual.evidence().remainder().is_some());
        assert_eq!(actual.evidence().end_terms().len(), order + 1);
    }
}
#[test]
fn all_prefixes_equal_fresh_programs_and_complete_proofs() {
    compare(
        &field::<1>(
            &[Expr::real(hyperreal::Real::from(2).sqrt().unwrap())],
            &[X],
        ),
        step([d(1, 1, 0)], [d(1, 2, 0)]),
    );
    compare(
        &field::<1>(&[Expr::anonymous_symbol(X)], &[X]),
        step([d(1, 1, 0)], [d(1, 2, 0)]),
    );
    compare(
        &field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T)], &[X]),
        step([d(0, 1, 0)], [d(0, 2, 0)]),
    );
    const Y: SymbolId = SymbolId(30);
    compare(
        &field::<2>(&[Expr::anonymous_symbol(Y), Expr::int(0)], &[X, Y]),
        step([d(0, 1, 0), d(1, 2, 0)], [d(0, 2, 0), d(1, 2, 0)]),
    );
}
#[test]
fn independent_polynomial_and_rational_solution_oracles_and_next_step() {
    let f = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T)], &[X]);
    let high = f.flow_taylor(4, &mut budget()).unwrap();
    let p = high.restrict_order(2, &mut budget()).unwrap();
    drop(high);
    let r = p.certify_step(step([d(3, 4, 0)], [d(3, 5, 0)]), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Certified));
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[d(193, 257, -6)]));
    assert_eq!(
        r.evidence().remainder().unwrap().lanes(),
        Some(&[d(0, 0, 0)])
    );
    let r2 = p.certify_step(
        PicardProposal {
            start: d(1, 1, -3),
            end: d(1, 1, -2),
            initial: r.end_enclosure().unwrap().clone(),
            candidate: BoxN::new([d(3, 5, 0)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(matches!(r2.status(), FlowTaylorStatus::Certified));
    assert_eq!(r2.end_enclosure().unwrap().lanes(), Some(&[d(49, 65, -4)]));
    let x = Expr::anonymous_symbol(X);
    let p = field::<1>(&[x.clone() * x], &[X])
        .flow_taylor(4, &mut budget())
        .unwrap()
        .restrict_order(3, &mut budget())
        .unwrap();
    let r = p.certify_step(step([d(1, 1, 0)], [d(1, 2, 0)]), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Certified));
    let exact = hyperreal::Rational::from(8) / hyperreal::Rational::from(7);
    assert!(
        r.end_enclosure().unwrap().lanes().unwrap()[0]
            .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
            .unwrap()
            .contains_point(&exact)
    );
}
#[test]
fn nested_restriction_retains_source_and_cannot_expand() {
    let high = field::<1>(&[Expr::int(2)], &[X])
        .flow_taylor(4, &mut budget())
        .unwrap();
    let equal = high.restrict_order(4, &mut budget()).unwrap();
    assert!(std::ptr::eq(
        equal.stages().as_ptr(),
        high.stages().as_ptr()
    ));
    let lower = equal.restrict_order(2, &mut budget()).unwrap();
    for requested in [3, usize::MAX] {
        assert!(
            matches!(lower.restrict_order(requested,&mut budget()),Err(FlowTaylorOrderError::UnpreparedOrder{requested:r,prepared:2}) if r==requested)
        );
    }
    let zero = lower.restrict_order(0, &mut budget()).unwrap();
    drop(high);
    drop(equal);
    drop(lower);
    let r = zero.certify_step(step([d(0, 0, 0)], [d(0, 1, 0)]), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Certified));
    assert!(r.evidence().coefficients().is_empty());
    assert_eq!(
        r.evidence().remainder().unwrap().lanes(),
        Some(&[d(2, 2, 0)])
    );
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[d(1, 1, -2)]));
}
#[test]
fn empty_unit_and_invalid_proposals_preserve_existing_semantics() {
    let p = field::<0>(&[], &[])
        .flow_taylor(3, &mut budget())
        .unwrap()
        .restrict_order(1, &mut budget())
        .unwrap();
    let unit = p.certify_step(step([], []), &mut budget());
    assert!(matches!(unit.status(), FlowTaylorStatus::Certified));
    assert_eq!(unit.end_enclosure().unwrap().lanes(), Some(&[]));
    let mut proposal = step([], []);
    proposal.initial = BoxN::empty().unwrap();
    let empty = p.certify_step(proposal.clone(), &mut budget());
    assert!(matches!(empty.status(), FlowTaylorStatus::Empty));
    assert!(empty.end_enclosure().unwrap().is_empty());
    proposal.end = d(-1, -1, 0);
    let invalid = p.certify_step(proposal, &mut budget());
    assert!(matches!(invalid.status(), FlowTaylorStatus::Rejected(_)));
    assert!(invalid.end_enclosure().is_none());
}
#[test]
fn restriction_does_not_erase_original_denominator_guards() {
    let f = field::<1>(&[Expr::int(0) / Expr::anonymous_symbol(X)], &[X]);
    let high = f.flow_taylor(3, &mut budget()).unwrap();
    for order in 0..=3 {
        let prefix = high.restrict_order(order, &mut budget()).unwrap();
        let proposal = step([d(0, 0, 0)], [d(-1, 1, 0)]);
        let r = prefix.certify_step(proposal.clone(), &mut budget());
        assert!(matches!(r.status(), FlowTaylorStatus::Rejected(_)));
        assert!(r.end_enclosure().is_none());
        let fresh = f
            .flow_taylor(order, &mut budget())
            .unwrap()
            .certify_step(proposal, &mut budget());
        assert_eq!(format!("{r:?}"), format!("{fresh:?}"));
    }
}
#[test]
fn structural_boundaries_cancel_and_retry_never_mutate_source() {
    let high = field::<1>(&[Expr::anonymous_symbol(X)], &[X])
        .flow_taylor(4, &mut budget())
        .unwrap();
    let before = format!("{high:?}");
    for order in 0..=4 {
        let mut b = budget();
        let p = high.restrict_order(order, &mut b).unwrap();
        let work = b.policy().graph_work - b.remaining_graph_work();
        assert_eq!(work, if order == 4 { 1 } else { 1 + 10 * (order + 1) });
        assert_eq!(b.remaining_scalar_work(), b.policy().scalar_work);
        let mut exact = EnclosureBudget::new(EnclosurePolicy {
            graph_work: work,
            scalar_work: 0,
            ..EnclosurePolicy::default()
        });
        assert_eq!(
            format!("{:?}", high.restrict_order(order, &mut exact).unwrap()),
            format!("{p:?}")
        );
        assert_eq!(exact.remaining_graph_work(), 0);
        let mut short = EnclosureBudget::new(EnclosurePolicy {
            graph_work: work - 1,
            ..EnclosurePolicy::default()
        });
        assert!(matches!(
            high.restrict_order(order, &mut short),
            Err(FlowTaylorOrderError::Resource(
                ExpressionError::WorkExhausted
            ))
        ));
    }
    let mut small = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: 1,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        high.restrict_order(0, &mut small),
        Err(FlowTaylorOrderError::Resource(
            ExpressionError::SizeExhausted
        ))
    ));
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut stopped = budget();
    stopped.set_cancellation(Some(token));
    for order in [0, 4, usize::MAX] {
        assert!(matches!(
            high.restrict_order(order, &mut stopped),
            Err(FlowTaylorOrderError::Resource(ExpressionError::Cancelled))
        ));
    }
    assert_eq!(format!("{high:?}"), before);
    assert_eq!(high.restrict_order(2, &mut budget()).unwrap().order(), 2);
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn restriction_has_zero_dispatch_and_step_dispatch_matches_fresh() {
    use hyperdynamics::field::taylor_flow::FlowTaylorProgram;
    use hyperreal::dispatch_trace;
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    let high = f.flow_taylor(4, &mut budget()).unwrap();
    for order in 0..=4 {
        dispatch_trace::reset();
        let prefix =
            dispatch_trace::with_recording(|| high.restrict_order(order, &mut budget()).unwrap());
        let trace = dispatch_trace::take_trace();
        assert!(trace.dispatch.is_empty());
        assert_eq!(trace.rational, Default::default());
        let fresh = f.flow_taylor(order, &mut budget()).unwrap();
        let proposal = step([d(1, 1, 0)], [d(1, 2, 0)]);
        let run = |p: &FlowTaylorProgram<1>| {
            dispatch_trace::reset();
            let r =
                dispatch_trace::with_recording(|| p.certify_step(proposal.clone(), &mut budget()));
            assert!(matches!(r.status(), FlowTaylorStatus::Certified));
            dispatch_trace::take_trace()
        };
        assert_eq!(run(&prefix), run(&fresh));
    }
}
#[cfg(feature = "taylor-evolution")]
#[test]
fn shared_order_is_consumed_by_whole_horizon_evolution() {
    use hyperdynamics::field::{
        evolution::{EvolutionOptions, EvolutionRequest},
        taylor_flow::evolution::FlowTaylorEvolutionStatus,
    };
    let f = field::<1>(&[Expr::int(2)], &[X]);
    let high = f.flow_taylor(5, &mut budget()).unwrap();
    let request = EvolutionRequest {
        start: d(0, 0, 0),
        end: d(1, 1, -1),
        initial: BoxN::new([d(0, 0, 0)]).unwrap(),
    };
    let options = EvolutionOptions {
        maximum_step: Some(d(1, 1, -2)),
        ..Default::default()
    };
    for order in 0..=3 {
        let shared = high
            .restrict_order(order, &mut budget())
            .unwrap()
            .integrate(request.clone(), options.clone(), &mut budget());
        let fresh = f.flow_taylor(order, &mut budget()).unwrap().integrate(
            request.clone(),
            options.clone(),
            &mut budget(),
        );
        assert!(matches!(
            shared.status(),
            FlowTaylorEvolutionStatus::Completed
        ));
        assert_eq!(shared.segments().len(), 2);
        assert_eq!(shared.end_enclosure().unwrap().lanes(), Some(&[d(1, 1, 0)]));
        assert_eq!(format!("{shared:?}"), format!("{fresh:?}"));
    }
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_source_and_order_restriction() {
    const KEY: &str = "HYPERDYNAMICS_ORDER_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, max, order, proposal): (FieldDefinition, usize, usize, PicardProposal<1>) =
            serde_json::from_str(&encoded).unwrap();
        let f = ExpressionField::<1>::from_definition(definition, &mut budget()).unwrap();
        let p = f
            .flow_taylor(max, &mut budget())
            .unwrap()
            .restrict_order(order, &mut budget())
            .unwrap();
        let r = p.certify_step(proposal, &mut budget());
        assert!(matches!(r.status(), FlowTaylorStatus::Certified));
        println!("REPLAY={r:?}");
        return;
    }
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X]);
    use hyperdynamics::field::EnclosedVectorField;
    let id = f.identity();
    let definition = FieldDefinition {
        source: id.source().clone(),
        time: id.time(),
        state: id.state().to_vec(),
    };
    let encoded =
        serde_json::to_string(&(definition, 4, 2, step([d(1, 1, 0)], [d(1, 2, 0)]))).unwrap();
    let mut prior = None;
    for _ in 0..3 {
        let output = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_rebuilds_source_and_order_restriction",
                "--nocapture",
            ])
            .env(KEY, &encoded)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let text = String::from_utf8(output.stdout).unwrap();
        let value = text
            .lines()
            .find(|s| s.starts_with("REPLAY="))
            .unwrap()
            .to_owned();
        if let Some(p) = prior.as_ref() {
            assert_eq!(&value, p);
        }
        prior = Some(value);
    }
}
#[path = "../examples/taylor_orders.rs"]
mod consumer;
#[test]
fn actual_example_reuses_preparation_for_consecutive_steps() {
    consumer::run().unwrap();
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
