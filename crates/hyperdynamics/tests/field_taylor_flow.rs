#![cfg(feature = "taylor-flow")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, picard::PicardProposal,
    taylor_flow::FlowTaylorStatus,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
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
fn field<const N: usize>(
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
fn exponential_flow_retains_remainder_and_accepts_its_endpoint() {
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X, T], &[X]);
    let program = f.flow_taylor(3, &mut budget()).unwrap();
    let proposal = PicardProposal {
        start: d(0, 0, 0),
        end: d(1, 1, -2),
        initial: BoxN::new([d(1, 1, 0)]).unwrap(),
        candidate: BoxN::new([d(1, 2, 0)]).unwrap(),
    };
    let report = program.certify_step(proposal, &mut budget());
    assert!(
        matches!(report.status(), FlowTaylorStatus::Certified),
        "{:?}",
        report.status()
    );
    assert_eq!(report.evidence().coefficients().len(), 3);
    assert!(report.evidence().remainder().is_some());
    // e^(1/4) lies between the exact rational partial sum through degree 6
    // and that sum plus the geometric upper bound on the remaining positive tail.
    let end = report.end_enclosure().unwrap().lanes().unwrap()[0]
        .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
        .unwrap();
    let q = hyperreal::Rational::from(4);
    let mut term = hyperreal::Rational::from(1);
    let mut lower = term.clone();
    for k in 1..=6 {
        term = term / (&q * hyperreal::Rational::from(k));
        lower = lower + &term;
    }
    let upper = &lower
        + (&term / hyperreal::Rational::from(28))
            / (hyperreal::Rational::from(1)
                - hyperreal::Rational::from(1) / hyperreal::Rational::from(32));
    assert!(end.contains_point(&lower));
    assert!(end.contains_point(&upper));
    let next = program.certify_step(
        PicardProposal {
            start: d(1, 1, -2),
            end: d(1, 1, -1),
            initial: report.end_enclosure().unwrap().clone(),
            candidate: BoxN::new([d(1, 2, 0)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(next.status(), FlowTaylorStatus::Certified),
        "{:?}",
        next.status()
    );
}

fn step<const N: usize>(
    initial: [DyadicInterval; N],
    candidate: [DyadicInterval; N],
) -> PicardProposal<N> {
    PicardProposal {
        start: d(0, 0, 0),
        end: d(1, 1, -2),
        initial: BoxN::new(initial).unwrap(),
        candidate: BoxN::new(candidate).unwrap(),
    }
}
#[test]
fn nonautonomous_polynomial_and_uncertain_triangular_flows_have_exact_ranges() {
    let f = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T)], &[T, X], &[X]);
    let p = f.flow_taylor(2, &mut budget()).unwrap();
    let r = p.certify_step(step([d(3, 3, 0)], [d(3, 4, 0)]), &mut budget());
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[d(49, 49, -4)]));
    assert_eq!(r.tube().unwrap().lanes(), Some(&[d(48, 49, -4)]));
    assert_eq!(r.evidence().coefficients()[0].lanes(), Some(&[d(0, 0, 0)]));
    assert_eq!(r.evidence().coefficients()[1].lanes(), Some(&[d(1, 1, 0)]));
    assert_eq!(
        r.evidence().remainder().unwrap().lanes(),
        Some(&[d(0, 0, 0)])
    );
    const Y: SymbolId = SymbolId(30);
    let f = field::<2>(
        &[Expr::anonymous_symbol(Y), Expr::int(0)],
        &[Y, T, X],
        &[X, Y],
    );
    let r = f.flow_taylor(2, &mut budget()).unwrap().certify_step(
        step([d(0, 1, 0), d(1, 2, 0)], [d(0, 2, 0), d(1, 2, 0)]),
        &mut budget(),
    );
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    assert_eq!(
        r.end_enclosure().unwrap().lanes(),
        Some(&[d(1, 6, -2), d(1, 2, 0)])
    );
    assert_eq!(
        r.evidence().remainder().unwrap().lanes(),
        Some(&[d(0, 0, 0), d(0, 0, 0)])
    );
}
#[test]
fn riccati_and_oscillator_enclose_independent_rational_expectations() {
    use hyperreal::Rational as Q;
    let x = Expr::anonymous_symbol(X);
    let f = field::<1>(&[x.clone() * x], &[T, X], &[X]);
    let mut proposal = step([d(1, 1, 0)], [d(1, 2, 0)]);
    proposal.end = d(1, 1, -3);
    let r = f
        .flow_taylor(3, &mut budget())
        .unwrap()
        .certify_step(proposal, &mut budget());
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    for c in r.evidence().coefficients() {
        assert_eq!(c.lanes(), Some(&[d(1, 1, 0)]));
    }
    let exact = Q::from(8) / Q::from(7);
    assert!(
        r.end_enclosure().unwrap().lanes().unwrap()[0]
            .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
            .unwrap()
            .contains_point(&exact)
    );
    const Y: SymbolId = SymbolId(30);
    let f = field::<2>(
        &[Expr::anonymous_symbol(Y), -Expr::anonymous_symbol(X)],
        &[Y, T, X],
        &[X, Y],
    );
    let r = f.flow_taylor(3, &mut budget()).unwrap().certify_step(
        step([d(1, 1, 0), d(0, 0, 0)], [d(1, 3, -1), d(-1, 1, -1)]),
        &mut budget(),
    );
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    let h = Q::from(1) / Q::from(4);
    let h2 = &h * &h;
    let mut cos = Q::from(1);
    let mut cterm = cos.clone();
    let mut sin = h.clone();
    let mut sterm = h;
    let mut previous = (cos.clone(), sin.clone());
    for k in 1..=6 {
        previous = (cos.clone(), sin.clone());
        cterm = -cterm * &h2 / Q::from((2 * k - 1) * (2 * k));
        cos = cos + &cterm;
        sterm = -sterm * &h2 / Q::from((2 * k) * (2 * k + 1));
        sin = sin + &sterm;
    }
    let lanes = r.end_enclosure().unwrap().lanes().unwrap();
    for (lane, expectations) in lanes.iter().zip([[previous.0, cos], [-previous.1, -sin]]) {
        let bound = lane
            .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
            .unwrap();
        for q in expectations {
            assert!(bound.contains_point(&q), "{bound:?} misses {q:?}");
        }
    }
}
#[test]
fn zero_order_empty_unit_and_invalid_proposals_remain_distinct() {
    use hyperdynamics::field::{picard::PicardFailure, taylor_flow::FlowTaylorError};
    let f = field::<1>(&[Expr::int(-2)], &[T, X], &[X]);
    let p = f.flow_taylor(0, &mut budget()).unwrap();
    let r = p.certify_step(step([d(1, 1, 0)], [d(0, 2, 0)]), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Certified));
    assert!(r.evidence().coefficients().is_empty());
    assert_eq!(
        r.evidence().remainder().unwrap().lanes(),
        Some(&[d(-2, -2, 0)])
    );
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[d(1, 1, -1)]));
    let empty = PicardProposal {
        initial: BoxN::empty().unwrap(),
        ..step([d(1, 1, 0)], [d(0, 2, 0)])
    };
    let r = p.certify_step(empty.clone(), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Empty));
    assert!(r.end_enclosure().unwrap().is_empty());
    assert!(r.evidence().analytic().is_none());
    let r = p.certify_step(
        PicardProposal {
            end: d(0, 0, 0),
            ..empty
        },
        &mut budget(),
    );
    assert!(matches!(
        r.status(),
        FlowTaylorStatus::Rejected(FlowTaylorError::Picard(PicardFailure::NonPositiveDuration))
    ));
    assert!(r.end_enclosure().is_none());
    let f = field::<0>(&[], &[T], &[]);
    let r = f
        .flow_taylor(3, &mut budget())
        .unwrap()
        .certify_step(step([], []), &mut budget());
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[]));
    assert!(r.evidence().analytic().is_some());
    assert!(matches!(
        f.flow_taylor(usize::MAX, &mut budget()),
        Err(FlowTaylorError::OrderOverflow)
    ));
}
#[test]
fn rejected_picard_and_singular_sources_never_publish_taylor_success() {
    use hyperdynamics::field::taylor_flow::FlowTaylorError;
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[T, X], &[X]);
    let r = f
        .flow_taylor(3, &mut budget())
        .unwrap()
        .certify_step(step([d(1, 1, 0)], [d(1, 1, 0)]), &mut budget());
    assert!(matches!(
        r.status(),
        FlowTaylorStatus::Rejected(FlowTaylorError::Picard(_))
    ));
    assert!(r.end_enclosure().is_none());
    assert!(r.evidence().analytic().is_none());
    let f = field::<1>(
        &[Expr::int(1)
            / (Expr::anonymous_symbol(T)
                - Expr::real(hyperreal::Real::from(
                    hyperreal::Rational::from(1) / hyperreal::Rational::from(8),
                )))],
        &[X, T],
        &[X],
    );
    let r = f
        .flow_taylor(2, &mut budget())
        .unwrap()
        .certify_step(step([d(0, 0, 0)], [d(-2, 2, 0)]), &mut budget());
    assert!(matches!(r.status(), FlowTaylorStatus::Rejected(_)));
    assert!(r.end_enclosure().is_none());
    assert!(r.tube().is_none());
}

#[test]
fn budgets_final_contribution_cancellation_and_retry_are_explicit() {
    use hyperdynamics::field::{FieldError, picard::PicardStatus, taylor_flow::FlowTaylorError};
    use hypersolve::expression_program::ExpressionError;
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X, T], &[X]);
    let mut measured = budget();
    let p = f.flow_taylor(3, &mut measured).unwrap();
    let prepare = EnclosurePolicy::default().graph_work - measured.remaining_graph_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: prepare - 1,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        f.flow_taylor(3, &mut short),
        Err(FlowTaylorError::Field(FieldError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: prepare,
        ..EnclosurePolicy::default()
    });
    assert_eq!(
        format!("{:?}", f.flow_taylor(3, &mut exact).unwrap()),
        format!("{p:?}")
    );
    assert_eq!(exact.remaining_graph_work(), 0);
    let proposal = step([d(1, 1, 0)], [d(1, 2, 0)]);
    for _ in 0..3 {
        assert!(matches!(
            p.certify_step(proposal.clone(), &mut budget()).status(),
            FlowTaylorStatus::Certified
        ));
    }
    let mut measured = budget();
    let good = p.certify_step(proposal.clone(), &mut measured);
    assert!(matches!(good.status(), FlowTaylorStatus::Certified));
    let work = EnclosurePolicy::default().graph_work - measured.remaining_graph_work();
    let scalar = EnclosurePolicy::default().scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: work,
        scalar_work: scalar,
        ..EnclosurePolicy::default()
    });
    let complete = p.certify_step(proposal.clone(), &mut exact);
    assert!(
        matches!(complete.status(), FlowTaylorStatus::Certified),
        "{:?}",
        complete.status()
    );
    assert_eq!(complete.end_enclosure(), good.end_enclosure());
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    for policy in [
        EnclosurePolicy {
            graph_work: work - 1,
            ..EnclosurePolicy::default()
        },
        EnclosurePolicy {
            scalar_work: scalar - 1,
            ..EnclosurePolicy::default()
        },
    ] {
        let report = p.certify_step(proposal.clone(), &mut EnclosureBudget::new(policy));
        assert!(matches!(report.picard().status(), PicardStatus::Certified));
        assert!(matches!(report.status(), FlowTaylorStatus::Rejected(_)));
        assert!(report.evidence().remainder().is_some());
        assert_eq!(report.evidence().end_terms().len(), 4);
        assert!(report.end_enclosure().is_none());
        assert!(report.tube().is_none());
    }
    let mut tiny = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: 1,
        ..EnclosurePolicy::default()
    });
    assert!(f.flow_taylor(3, &mut tiny).is_err());
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut stopped = budget();
    stopped.set_cancellation(Some(token));
    assert!(f.flow_taylor(3, &mut stopped).is_err());
    assert!(matches!(
        p.certify_step(proposal.clone(), &mut stopped).status(),
        FlowTaylorStatus::Rejected(_)
    ));
    assert!(matches!(
        p.certify_step(proposal, &mut budget()).status(),
        FlowTaylorStatus::Certified
    ));
    let constant = field::<1>(&[Expr::int(2)], &[X, T], &[X]);
    let high = constant.flow_taylor(8, &mut budget()).unwrap();
    assert_eq!(high.stages().len(), 9);
    let r = high.certify_step(step([d(0, 0, 0)], [d(0, 1, 0)]), &mut budget());
    assert!(
        matches!(r.status(), FlowTaylorStatus::Certified),
        "{:?}",
        r.status()
    );
    assert_eq!(r.end_enclosure().unwrap().lanes(), Some(&[d(1, 1, -1)]));
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn evidence_access_performs_no_predicate_or_rational_dispatch() {
    use hyperreal::dispatch_trace;
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X, T], &[X]);
    let p = f.flow_taylor(3, &mut budget()).unwrap();
    let r = p.certify_step(step([d(1, 1, 0)], [d(1, 2, 0)]), &mut budget());
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        assert_eq!(r.program().order(), 3);
        assert_eq!(r.program().stages().len(), 4);
        for stage in r.program().stages() {
            assert_eq!(stage.outputs().len(), 1);
            let _ = stage.program();
            let _ = stage.derivatives();
            assert!(stage.degree() > 0);
        }
        let _ = r.program().identity();
        let _ = r.picard().identity();
        assert!(r.evidence().analytic().is_some());
        assert_eq!(r.evidence().coefficients().len(), 3);
        assert!(r.evidence().remainder().is_some());
        assert_eq!(r.evidence().end_terms().len(), 4);
        assert_eq!(r.evidence().tube_terms().len(), 4);
        assert!(r.end_enclosure().is_some());
        assert!(r.tube().is_some());
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_regenerates_the_complete_flow_evidence() {
    use hyperdynamics::field::EnclosedVectorField;
    const KEY: &str = "HYPERDYNAMICS_TAYLOR_FLOW_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, order, proposal): (FieldDefinition, usize, PicardProposal<1>) =
            serde_json::from_str(&encoded).unwrap();
        let mut budget = budget();
        let f = ExpressionField::<1>::from_definition(definition, &mut budget).unwrap();
        let p = f.flow_taylor(order, &mut budget).unwrap();
        let r = p.certify_step(proposal, &mut budget);
        assert!(
            matches!(r.status(), FlowTaylorStatus::Certified),
            "{:?}",
            r.status()
        );
        println!(
            "FLOW {r:?} work {} {}",
            budget.remaining_graph_work(),
            budget.remaining_scalar_work()
        );
        return;
    }
    let f = field::<1>(&[Expr::anonymous_symbol(X)], &[X, T], &[X]);
    let definition = FieldDefinition {
        source: f.identity().source().clone(),
        time: T,
        state: vec![X],
    };
    let encoded =
        serde_json::to_string(&(definition.clone(), 3usize, step([d(1, 1, 0)], [d(1, 2, 0)])))
            .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let output = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_regenerates_the_complete_flow_evidence",
                "--nocapture",
            ])
            .env(KEY, &encoded)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        let text = String::from_utf8(output.stdout).unwrap();
        let row = text
            .lines()
            .find(|l| l.starts_with("FLOW "))
            .unwrap()
            .to_owned();
        if let Some(old) = previous {
            assert_eq!(row, old);
        }
        previous = Some(row);
    }
    let mut invalid = definition.clone();
    invalid.state[0] = T;
    assert!(ExpressionField::<1>::from_definition(invalid, &mut budget()).is_err());
    let mut invalid = definition;
    invalid.source.outputs[0] = usize::MAX;
    assert!(ExpressionField::<1>::from_definition(invalid, &mut budget()).is_err());
}

#[test]
fn a_picard_time_continuity_proof_does_not_assert_time_analyticity() {
    use hyperdynamics::field::{picard::PicardStatus, taylor_flow::FlowTaylorError};
    let f = field::<1>(&[Expr::anonymous_symbol(T).sqrt()], &[T, X], &[X]);
    let proposal = step([d(0, 0, 0)], [d(0, 1, 0)]);
    let picard = f.certify_step(proposal.clone(), &mut budget());
    assert!(
        matches!(picard.status(), PicardStatus::Certified),
        "{:?}",
        picard.status()
    );
    let report = f
        .flow_taylor(2, &mut budget())
        .unwrap()
        .certify_step(proposal, &mut budget());
    assert!(matches!(report.picard().status(), PicardStatus::Certified));
    assert!(
        matches!(
            report.status(),
            FlowTaylorStatus::Rejected(FlowTaylorError::Analytic(_))
        ),
        "{:?}",
        report.status()
    );
    assert!(report.end_enclosure().is_none());
    assert!(report.tube().is_none());
}

#[test]
fn joint_time_state_lie_terms_match_exact_independent_coefficients() {
    use hyperreal::{Rational as Q, Real};
    let f = field::<1>(
        &[Expr::anonymous_symbol(T) * Expr::anonymous_symbol(X)],
        &[X, T],
        &[X],
    );
    let p = f.flow_taylor(3, &mut budget()).unwrap();
    let expected = [
        Q::from(2),
        Q::from(2),
        Q::from(4) / Q::from(3),
        Q::from(5) / Q::from(6),
    ];
    for (stage, coefficient) in p.stages().iter().zip(expected) {
        let values = stage
            .program()
            .eval_real(
                &[Real::from(2), Real::from(1)],
                &mut ExpressionBudget::new(1_000_000, 100_000_000),
            )
            .unwrap();
        assert_eq!(values[stage.outputs()[0]], Real::from(coefficient));
    }
    let report = p.certify_step(
        PicardProposal {
            start: d(1, 1, 0),
            end: d(5, 5, -2),
            initial: BoxN::new([d(2, 2, 0)]).unwrap(),
            candidate: BoxN::new([d(2, 4, 0)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(report.status(), FlowTaylorStatus::Certified),
        "{:?}",
        report.status()
    );
    // x(t)=2 exp((t^2-1)/2), so x(5/4)=2 exp(9/32).
    let z = Q::from(9) / Q::from(32);
    let mut term = Q::from(1);
    let mut lower = term.clone();
    for k in 1..=10 {
        term = term * &z / Q::from(k);
        lower = lower + &term;
    }
    let upper = &lower + (&term * &z / Q::from(11)) / (Q::from(1) - &z / Q::from(12));
    let bounds = report.end_enclosure().unwrap().lanes().unwrap()[0]
        .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
        .unwrap();
    assert!(bounds.contains_point(&(lower * Q::from(2))));
    assert!(bounds.contains_point(&(upper * Q::from(2))));
}

#[cfg(feature = "coordinates")]
#[test]
fn exact_state_chart_and_rational_clock_consume_the_flow_endpoint() {
    use hyperdynamics::field::EnclosedVectorField;
    use hyperreal::Rational as Q;
    use hypersolve::expression_program::CoordinateChange;
    let original = field::<1>(&[Expr::int(2)], &[X, T], &[X]);
    let chart = original
        .in_coordinates(
            CoordinateChange {
                variables: vec![X],
                to_source: vec![vec![2.into()]],
                from_source: vec![vec![(hyperreal::Real::from(Q::from(1) / Q::from(2)))]],
                origin: vec![10.into()],
            },
            &mut budget(),
        )
        .unwrap();
    let clocked = chart
        .transformed_field()
        .on_time_span(
            Q::from(1) / Q::from(3),
            Q::from(2) / Q::from(3),
            &mut budget(),
        )
        .unwrap();
    let program = clocked
        .normalized_field()
        .flow_taylor(2, &mut budget())
        .unwrap();
    assert_eq!(program.identity(), clocked.normalized_field().identity());
    let result = program.certify_step(
        PicardProposal {
            start: d(0, 0, 0),
            end: d(1, 1, 0),
            initial: BoxN::new([d(0, 0, 0)]).unwrap(),
            candidate: BoxN::new([d(0, 1, 0)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(result.status(), FlowTaylorStatus::Certified),
        "{:?}",
        result.status()
    );
    let physical = chart
        .source_enclosure(result.end_enclosure().unwrap(), &mut budget())
        .unwrap();
    assert!(
        physical.lanes().unwrap()[0]
            .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
            .unwrap()
            .contains_point(&(Q::from(32) / Q::from(3)))
    );
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
