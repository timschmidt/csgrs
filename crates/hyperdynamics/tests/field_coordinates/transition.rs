use super::*;
use hyperdynamics::field::coordinates::{CoordinateFieldTransition, CoordinateTransportError};
fn identity() -> CoordinateChange {
    CoordinateChange {
        variables: vec![X, Y],
        to_source: vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]],
        from_source: vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]],
        origin: vec![0.into(), 0.into()],
    }
}
fn wide() -> CoordinateField<2> {
    let mut c = change();
    c.to_source[0][1] = 128.into();
    c.from_source[0][1] = (-128).into();
    field(&[Expr::int(129), Expr::int(1)], vec![X, Y])
        .in_coordinates(c, &mut budget())
        .unwrap()
}
#[test]
fn direct_chart_transport_avoids_cartesian_roundtrip_wrapping() {
    let a = wide();
    let b = a
        .original_field()
        .in_coordinates(a.transformation().change().clone(), &mut budget())
        .unwrap();
    let physical = a
        .original_field()
        .in_coordinates(identity(), &mut budget())
        .unwrap();
    let initial = BoxN::new([d(0, 0), d(-1, 1)]).unwrap();
    let direct = a
        .transition_to(&b, &mut budget())
        .unwrap()
        .enclose(initial.clone(), &mut budget())
        .unwrap();
    assert_eq!(direct.enclosure(), &initial);
    assert_eq!(direct.initial(), &initial);
    let projected = a
        .transition_to(&physical, &mut budget())
        .unwrap()
        .enclose(initial.clone(), &mut budget())
        .unwrap();
    assert_eq!(
        projected.enclosure().lanes(),
        Some(&[d(-118, 138), d(-4, -2)])
    );
    let roundtrip = physical
        .transition_to(&b, &mut budget())
        .unwrap()
        .enclose(projected.enclosure().clone(), &mut budget())
        .unwrap();
    assert_eq!(
        roundtrip.enclosure().lanes(),
        Some(&[d(-256, 256), d(-1, 1)])
    );
    assert!(
        direct
            .enclosure()
            .subset_of(
                roundtrip.enclosure(),
                &mut hyperinterval::DyadicBudget::new(16384, 1_000_000)
            )
            .unwrap()
    );
}
#[test]
fn generated_endpoint_changes_chart_and_is_consumed_by_next_step() {
    let a = chart(); // x=10+y0+y1, y=-3+y1, native derivative (1,1).
    let first = a.certify_step(proposal(), &mut budget());
    assert!(matches!(first.status(), CoordinateStatus::Certified));
    let mut c = change();
    c.origin = vec![12.into(), (-2).into()];
    let b = a.original_field().in_coordinates(c, &mut budget()).unwrap();
    let transition = a.transition_to(&b, &mut budget()).unwrap();
    let transported = transition
        .enclose(
            first.native().end_enclosure().unwrap().clone(),
            &mut budget(),
        )
        .unwrap();
    assert_eq!(transported.enclosure().lanes(), Some(&[d(-1, 1), d(-1, 1)]));
    let next = transported.transition().to().certify_step(
        PicardProposal {
            start: d(1, 1),
            end: d(2, 2),
            initial: transported.enclosure().clone(),
            candidate: BoxN::new([d(-1, 2), d(-1, 2)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(
        matches!(next.status(), CoordinateStatus::Certified),
        "{:?}",
        next.status()
    );
    // Exact source flow from the original correlated set at t=2: (14+a+b,-1+b).
    assert_eq!(
        next.source_end_enclosure().unwrap().lanes(),
        Some(&[d(12, 16), d(-2, 0)])
    );
    let again = b
        .transition_to(&a, &mut budget())
        .unwrap()
        .enclose(
            next.native().end_enclosure().unwrap().clone(),
            &mut budget(),
        )
        .unwrap();
    assert_eq!(again.enclosure().lanes(), Some(&[d(1, 3), d(1, 3)]));
}
#[test]
fn empty_unit_foreign_fields_and_time_interpretation() {
    let a = chart();
    let m = a.transition_to(&a, &mut budget()).unwrap();
    assert!(
        m.enclose(BoxN::empty().unwrap(), &mut budget())
            .unwrap()
            .enclosure()
            .is_empty()
    );
    let p = field::<0>(&[], vec![])
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
    let u = p
        .transition_to(&p, &mut budget())
        .unwrap()
        .enclose(BoxN::new([]).unwrap(), &mut budget())
        .unwrap();
    assert!(!u.enclosure().is_empty());
    let other = field(&[Expr::int(4), Expr::int(1)], vec![X, Y])
        .in_coordinates(change(), &mut budget())
        .unwrap();
    assert!(matches!(
        a.transition_to(&other, &mut budget()),
        Err(CoordinateTransportError::Transition(_))
    ));
    let id = a.original_field().identity();
    let f = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: id.source().clone(),
            time: X,
            state: vec![T, Y],
        },
        &mut budget(),
    )
    .unwrap();
    let mut c = change();
    c.variables = vec![T, Y];
    let other = f.in_coordinates(c, &mut budget()).unwrap();
    assert!(matches!(
        a.transition_to(&other, &mut budget()),
        Err(CoordinateTransportError::InterpretationMismatch)
    ));
}
fn prepared() -> CoordinateFieldTransition<2> {
    let a = chart();
    a.transition_to(&a, &mut budget()).unwrap()
}
#[test]
fn transport_final_work_scalar_limits_cancellation_and_retry() {
    let m = prepared();
    let initial = proposal().initial;
    let mut full = budget();
    m.enclose(initial.clone(), &mut full).unwrap();
    let used = EnclosurePolicy::default().graph_work - full.remaining_graph_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used - 1,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        m.enclose(initial.clone(), &mut short),
        Err(CoordinateTransportError::Field(
            hyperdynamics::field::FieldError::Expression(
                hypersolve::expression_program::ExpressionError::WorkExhausted
            )
        ))
    ));
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used,
        ..EnclosurePolicy::default()
    });
    assert_eq!(
        m.enclose(initial.clone(), &mut exact).unwrap().enclosure(),
        &initial
    );
    assert_eq!(exact.remaining_graph_work(), 0);
    let scalar_used = EnclosurePolicy::default().scalar_work - full.remaining_scalar_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        scalar_work: scalar_used - 1,
        ..EnclosurePolicy::default()
    });
    assert!(m.enclose(initial.clone(), &mut short).is_err());
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        scalar_work: scalar_used,
        ..EnclosurePolicy::default()
    });
    m.enclose(initial.clone(), &mut exact).unwrap();
    assert_eq!(exact.remaining_scalar_work(), 0);
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token));
    assert!(m.enclose(initial.clone(), &mut cancelled).is_err());
    assert_eq!(
        m.enclose(initial.clone(), &mut budget())
            .unwrap()
            .enclosure(),
        &initial
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_transition_replay_rechecks_sources_and_inverses() {
    use hyperdynamics::field::coordinates::CoordinateFieldDefinition;
    const KEY: &str = "HYPERDYNAMICS_CHART_TRANSITION_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (a, b, input): (
            CoordinateFieldDefinition,
            CoordinateFieldDefinition,
            BoxN<DyadicInterval, 2>,
        ) = serde_json::from_str(&data).unwrap();
        let mut invalid = b.clone();
        invalid.change.from_source[0][1] = 0.into();
        assert!(CoordinateField::<2>::from_definition(invalid, &mut budget()).is_err());
        let a = CoordinateField::<2>::from_definition(a, &mut budget()).unwrap();
        let b = CoordinateField::<2>::from_definition(b, &mut budget()).unwrap();
        let mut work = budget();
        let result = a
            .transition_to(&b, &mut work)
            .unwrap()
            .enclose(input, &mut work)
            .unwrap();
        assert_eq!(result.enclosure().lanes(), Some(&[d(0, 0), d(-1, 1)]));
        println!(
            "TRANSITION {:?} GRAPH {} SCALAR {}",
            result,
            work.remaining_graph_work(),
            work.remaining_scalar_work()
        );
        return;
    }
    let a = wide();
    let id = a.original_field().identity();
    let definition = CoordinateFieldDefinition {
        source: FieldDefinition {
            source: id.source().clone(),
            time: id.time(),
            state: id.state().to_vec(),
        },
        change: a.transformation().change().clone(),
    };
    let data = serde_json::to_string(&(
        definition.clone(),
        definition,
        BoxN::new([d(0, 0), d(-1, 1)]).unwrap(),
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "transition::fresh_process_transition_replay_rechecks_sources_and_inverses",
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
            .find(|l| l.starts_with("TRANSITION "))
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
fn transport_getters_do_not_rediscover_scalar_proofs() {
    use hyperreal::dispatch_trace;
    let m = prepared();
    let proof = m.enclose(proposal().initial, &mut budget()).unwrap();
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        std::hint::black_box(proof.initial());
        std::hint::black_box(proof.enclosure());
        std::hint::black_box(proof.policy());
        let m = proof.transition();
        std::hint::black_box(m.from());
        std::hint::black_box(m.to());
        let r = m.relation();
        std::hint::black_box(r.from());
        std::hint::black_box(r.to());
        std::hint::black_box(r.matrix());
        std::hint::black_box(r.origin());
        std::hint::black_box(r.program());
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[cfg(feature = "taylor-flow")]
#[test]
fn taylor_endpoint_survives_transport_and_further_taylor_step() {
    use hyperdynamics::field::taylor_flow::FlowTaylorStatus;
    let a = chart();
    let t = a.transformed_field().flow_taylor(2, &mut budget()).unwrap();
    let first = t.certify_step(proposal(), &mut budget());
    assert!(matches!(first.status(), FlowTaylorStatus::Certified));
    let mut c = change();
    c.origin = vec![12.into(), (-2).into()];
    let b = a.original_field().in_coordinates(c, &mut budget()).unwrap();
    let proof = a
        .transition_to(&b, &mut budget())
        .unwrap()
        .enclose(first.end_enclosure().unwrap().clone(), &mut budget())
        .unwrap();
    let next = b
        .transformed_field()
        .flow_taylor(2, &mut budget())
        .unwrap()
        .certify_step(
            PicardProposal {
                start: d(1, 1),
                end: d(2, 2),
                initial: proof.enclosure().clone(),
                candidate: BoxN::new([d(-1, 2), d(-1, 2)]).unwrap(),
            },
            &mut budget(),
        );
    assert!(matches!(next.status(), FlowTaylorStatus::Certified));
    assert_eq!(
        b.source_enclosure(next.end_enclosure().unwrap(), &mut budget())
            .unwrap()
            .lanes(),
        Some(&[d(12, 16), d(-2, 0)])
    );
}
