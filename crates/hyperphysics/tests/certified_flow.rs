#![cfg(feature = "certified-flow")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, FieldError,
    evolution::{EvolutionOptions, EvolutionRequest, EvolutionStatus},
    picard::{PicardFailure, PicardProposal, PicardStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperlimit::{Certainty, PredicateOutcome, Sign};
use hyperphysics::{
    ForceAccumulator3, ForceContribution3, Real, SourceSpec, StepReplayReport3, Vector3,
    dynamics::{DynamicsError, ForceFieldDefinition3, PointMassDynamics3},
};
use hypersolve::{
    Expr, SymbolId, expression_program::ExpressionError, work_budget::CancellationToken,
};
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn point(a: i64) -> DyadicInterval {
    d(a, a, 0)
}
fn zero() -> BoxN<DyadicInterval, 6> {
    BoxN::new(std::array::from_fn(|_| point(0))).unwrap()
}
fn force() -> ForceAccumulator3 {
    let mut forces = ForceAccumulator3::default();
    forces.push(ForceContribution3 {
        source: "actuator".into(),
        force: Vector3::new([Real::from(3), Real::from(-2), Real::zero()]),
    });
    forces.push(ForceContribution3 {
        source: "bias".into(),
        force: Vector3::new([Real::from(1), Real::from(-2), Real::zero()]),
    });
    forces
}
fn proposal() -> PicardProposal<6> {
    PicardProposal {
        start: point(0),
        end: d(1, 1, -2),
        initial: zero(),
        candidate: BoxN::new([
            d(0, 1, -3),
            d(-1, 0, -3),
            point(0),
            d(0, 1, -1),
            d(-1, 0, -1),
            point(0),
        ])
        .unwrap(),
    }
}
fn definition(
    outputs: &[Expr],
    bindings: &[SymbolId],
    b: &mut EnclosureBudget,
) -> ForceFieldDefinition3 {
    let source = b.graph_budget().compile(outputs, bindings).unwrap();
    ForceFieldDefinition3 {
        source: source.definition().clone(),
        time: SymbolId(0),
        position: [SymbolId(1), SymbolId(2), SymbolId(3)],
        velocity: [SymbolId(4), SymbolId(5), SymbolId(6)],
        provenance: vec![SourceSpec::new("test force law", "exact expression")],
    }
}
fn xyz() -> [SymbolId; 7] {
    std::array::from_fn(|i| SymbolId(i as u32))
}
fn assert_contains(value: &DyadicInterval, n: i64, exponent: i32) {
    assert!(
        d(n, n, exponent)
            .subset_of(value, &mut DyadicBudget::new(4096, 100_000_000))
            .unwrap()
    );
}

#[test]
fn constant_force_flow_retains_all_sources_and_differs_from_discrete_replay() {
    let mut forces = force();
    let model = forces.dynamics(Real::from(2), &mut budget()).unwrap();
    assert_eq!(
        model.constant_forces().unwrap().contributions(),
        forces.contributions()
    );
    assert_eq!(model.provenance().len(), 2);
    assert!(matches!(
        model.mass_sign(),
        PredicateOutcome::Decided {
            value: Sign::Positive,
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        }
    ));
    forces.push(ForceContribution3 {
        source: "later mutation".into(),
        force: Vector3::new(std::array::from_fn(|_| Real::from(100))),
    });
    assert_eq!(model.constant_forces().unwrap().contributions().len(), 2);
    let report = model.certify_step(proposal(), &mut budget());
    assert!(
        matches!(report.native().status(), PicardStatus::Certified),
        "{:?}",
        report.native().status()
    );
    assert_eq!(
        report.native().identity(),
        &report.model().field().identity()
    );
    let end = report.native().end_enclosure().unwrap().lanes().unwrap();
    assert_eq!(&end[3..], &[d(1, 1, -1), d(-1, -1, -1), point(0)]);
    // Independent continuous x=(F/(2m))*t^2, v=(F/m)*t at t=1/4.
    assert_contains(&end[0], 1, -4);
    assert_contains(&end[1], -1, -4);
    assert_eq!(end[2], point(0));
    let replay = StepReplayReport3::symplectic_euler_replay(
        Real::from(2),
        (Real::one() / Real::from(4)).unwrap(),
        Vector3::zero(),
        Vector3::zero(),
        model.constant_forces().unwrap(),
    )
    .unwrap();
    assert_eq!(
        replay.proposed_position[0],
        (Real::one() / Real::from(8)).unwrap()
    );
    assert_ne!(
        replay.proposed_position[0],
        (Real::one() / Real::from(16)).unwrap()
    );
    // The ODE report encloses a continuous trajectory; old exact Euler replay still
    // denotes its original discrete method and is never used as this proof.
    assert!(replay.exact_replay);
}

#[test]
fn uncertain_initial_states_compose_and_whole_horizon_retains_attempts() {
    let model = force().dynamics(Real::from(2), &mut budget()).unwrap();
    let mut p = proposal();
    p.initial = BoxN::new([
        d(-1, 1, -4),
        point(0),
        point(0),
        d(-1, 1, -3),
        point(0),
        point(0),
    ])
    .unwrap();
    p.candidate = BoxN::new([
        d(-1, 1, 0),
        d(-1, 1, 0),
        point(0),
        d(-1, 1, 0),
        d(-1, 1, 0),
        point(0),
    ])
    .unwrap();
    let first = model.certify_step(p.clone(), &mut budget());
    assert!(matches!(first.native().status(), PicardStatus::Certified));
    let mut next = p;
    next.start = d(1, 1, -2);
    next.end = d(1, 1, -1);
    next.initial = first.native().end_enclosure().unwrap().clone();
    next.candidate = BoxN::new([
        d(-2, 2, 0),
        d(-2, 2, 0),
        point(0),
        d(-2, 2, 0),
        d(-2, 2, 0),
        point(0),
    ])
    .unwrap();
    let second = model.certify_step(next, &mut budget());
    assert!(matches!(second.native().status(), PicardStatus::Certified));
    let end = second.native().end_enclosure().unwrap().lanes().unwrap();
    for x0 in [-1_i64, 1] {
        for v0 in [-1_i64, 1] {
            // x0/16+(v0/8)*(1/2)+(1/2)^2; v0/8+2*(1/2).
            assert_contains(&end[0], x0 + v0 + 4, -4);
            assert_contains(&end[3], v0 + 8, -3);
        }
    }
    let whole = model.integrate(
        EvolutionRequest {
            start: point(0),
            end: d(1, 1, -1),
            initial: zero(),
        },
        EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..EvolutionOptions::default()
        },
        &mut budget(),
    );
    assert!(
        matches!(whole.native().status(), EvolutionStatus::Completed),
        "{:?}",
        whole.native().status()
    );
    assert!(whole.native().attempts().len() >= whole.native().segments().len());
    assert_eq!(whole.native().segments().len(), 2);
    assert_eq!(whole.model().mass(), &Real::from(2));
    let end = whole.native().end_enclosure().unwrap().lanes().unwrap();
    assert_contains(&end[0], 1, -2);
    assert_contains(&end[1], -1, -2);
    assert_eq!(end[3], point(1));
}

#[test]
fn nonlinear_time_velocity_forces_and_permuted_bindings_have_exact_source_meaning() {
    let v = Expr::anonymous_symbol(SymbolId(4));
    let x = Expr::anonymous_symbol(SymbolId(1));
    let t = Expr::anonymous_symbol(SymbolId(0));
    let outputs = [-x.clone() - v.clone() + t, x.clone() * x, Expr::int(0)];
    for shift in 0..7 {
        let mut bindings = xyz();
        bindings.rotate_left(shift);
        let model = PointMassDynamics3::new(
            Real::from(2),
            definition(&outputs, &bindings, &mut budget()),
            &mut budget(),
        )
        .unwrap();
        assert_eq!(model.force_source().definition().variables, bindings);
        let state =
            BoxN::new([point(2), point(0), point(0), point(3), point(0), point(0)]).unwrap();
        let image = model
            .field()
            .image(&point(1), &state, &mut budget())
            .unwrap();
        assert_eq!(
            image.lanes(),
            Some(&[point(3), point(0), point(0), point(-2), point(2), point(0)])
        );
        let jac = model
            .field()
            .jacobian(&point(1), &state, &mut budget())
            .unwrap();
        let mut expected =
            std::array::from_fn::<_, 6, _>(|_| std::array::from_fn::<_, 6, _>(|_| point(0)));
        expected[0][3] = point(1);
        expected[1][4] = point(1);
        expected[2][5] = point(1);
        expected[3][0] = d(-1, -1, -1);
        expected[3][3] = d(-1, -1, -1);
        expected[4][0] = point(2);
        assert_eq!(jac.rows(), Some(&expected));
        assert!(model.constant_forces().is_none());
    }
}

#[test]
fn original_invalid_refs_and_domain_guards_do_not_gain_validity_from_appended_nodes() {
    let mut def = definition(
        &std::array::from_fn::<_, 3, _>(|_| Expr::int(0)),
        &xyz(),
        &mut budget(),
    );
    def.source.outputs[0] = def.source.nodes.len();
    assert!(matches!(
        PointMassDynamics3::new(Real::one(), def, &mut budget()),
        Err(DynamicsError::Expression(
            ExpressionError::InvalidOutput { .. }
        ))
    ));
    let x = Expr::anonymous_symbol(SymbolId(1));
    let def = definition(
        &[Expr::int(1) / x, Expr::int(0), Expr::int(0)],
        &xyz(),
        &mut budget(),
    );
    let model = PointMassDynamics3::new(Real::one(), def, &mut budget()).unwrap();
    let failed = model.certify_step(proposal(), &mut budget());
    assert!(matches!(
        failed.native().status(),
        PicardStatus::Rejected(PicardFailure::Field(_))
    ));
    assert!(failed.native().end_enclosure().is_none());
}

#[test]
fn invalid_masses_shapes_empty_states_and_times_are_distinct() {
    for mass in [Real::zero(), Real::from(-1)] {
        assert!(matches!(
            force().dynamics(mass, &mut budget()),
            Err(DynamicsError::MassSign(PredicateOutcome::Decided {
                value: Sign::Zero | Sign::Negative,
                ..
            }))
        ));
    }
    let def = definition(&[Expr::int(0)], &xyz(), &mut budget());
    assert!(matches!(
        PointMassDynamics3::new(Real::one(), def, &mut budget()),
        Err(DynamicsError::ForceOutputCount { actual: 1 })
    ));
    let mut def = definition(
        &std::array::from_fn::<_, 3, _>(|_| Expr::int(0)),
        &xyz(),
        &mut budget(),
    );
    def.velocity[0] = def.position[0];
    assert!(matches!(
        PointMassDynamics3::new(Real::one(), def, &mut budget()),
        Err(DynamicsError::Field(FieldError::DuplicateState(_)))
    ));
    let model = force().dynamics(Real::one(), &mut budget()).unwrap();
    let mut p = proposal();
    p.initial = BoxN::empty().unwrap();
    let empty = model.certify_step(p.clone(), &mut budget());
    assert!(matches!(empty.native().status(), PicardStatus::Empty));
    assert!(empty.native().end_enclosure().unwrap().is_empty());
    p.end = p.start.clone();
    let invalid = model.certify_step(p, &mut budget());
    assert!(matches!(
        invalid.native().status(),
        PicardStatus::Rejected(PicardFailure::NonPositiveDuration)
    ));
    assert!(invalid.native().end_enclosure().is_none());
}

#[test]
fn force_provenance_shares_the_graph_entry_limit_before_construction() {
    let mut forces = definition(
        &[Expr::int(0), Expr::int(0), Expr::int(0)],
        &xyz(),
        &mut budget(),
    );
    forces.provenance = (0..1024)
        .map(|i| SourceSpec::new("force data", i.to_string()))
        .collect();
    // Seven variables, three constant nodes, three output references, seven
    // added mass/velocity/quotient nodes, six state outputs and all citations.
    let required = 7 + 3 + 3 + 7 + 6 + forces.provenance.len();
    let policy = EnclosurePolicy {
        graph_entries: required - 1,
        ..EnclosurePolicy::default()
    };
    let mut small = EnclosureBudget::new(policy);
    assert!(matches!(
        PointMassDynamics3::new(Real::one(), forces.clone(), &mut small),
        Err(DynamicsError::Expression(ExpressionError::SizeExhausted))
    ));
    assert_eq!(small.remaining_graph_work(), policy.graph_work);
    assert_eq!(small.remaining_scalar_work(), policy.scalar_work);

    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_entries: required,
        ..policy
    });
    let model = PointMassDynamics3::new(Real::one(), forces.clone(), &mut exact).unwrap();
    assert_eq!(model.provenance(), forces.provenance);
    assert_eq!(model.force_source().definition(), &forces.source);
    assert_eq!(
        model.field().image(&point(0), &zero(), &mut exact).unwrap(),
        zero()
    );
}

#[test]
fn shared_graph_budget_construction_cutoffs_cancelled_attempts_and_retry() {
    let forces = force();
    let mut measured = budget();
    measured.graph_budget().charge(37).unwrap();
    assert_eq!(
        measured.remaining_graph_work(),
        EnclosurePolicy::default().graph_work - 37
    );
    let model = forces.dynamics(Real::from(2), &mut measured).unwrap();
    let spent = EnclosurePolicy::default().graph_work - measured.remaining_graph_work() - 37;
    let mut small = EnclosureBudget::new(EnclosurePolicy {
        graph_work: spent - 1,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        forces.dynamics(Real::from(2), &mut small),
        Err(DynamicsError::Expression(ExpressionError::WorkExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.set_cancellation(Some(token));
    assert!(matches!(
        forces.dynamics(Real::from(2), &mut cancelled),
        Err(DynamicsError::Expression(ExpressionError::Cancelled))
    ));
    let rejected = model.certify_step(proposal(), &mut cancelled);
    assert!(matches!(
        rejected.native().status(),
        PicardStatus::Rejected(_)
    ));
    assert!(rejected.native().end_enclosure().is_none());
    assert!(matches!(
        model
            .certify_step(proposal(), &mut budget())
            .native()
            .status(),
        PicardStatus::Certified
    ));
    assert_eq!(
        model.constant_forces().unwrap().contributions(),
        forces.contributions()
    );
}

#[test]
fn algebraic_mass_is_authoritative_and_approximate_mass_signs_are_not_admitted() {
    let root = Real::from(2).sqrt().unwrap();
    let model = force().dynamics(root.clone(), &mut budget()).unwrap();
    assert_eq!(model.mass(), &root);
    let image = model
        .field()
        .image(&point(0), &zero(), &mut budget())
        .unwrap();
    let acceleration = image.lanes().unwrap()[3]
        .to_rational(&mut DyadicBudget::new(4096, 100_000_000))
        .unwrap();
    // 4/sqrt(2) has positive square 8, independently of the emitted expression.
    let lo = acceleration.lower().unwrap();
    let hi = acceleration.upper().unwrap();
    assert!(lo > &hyperreal::Rational::from(0));
    assert!(lo.clone() * lo.clone() <= hyperreal::Rational::from(8));
    assert!(hi.clone() * hi.clone() >= hyperreal::Rational::from(8));
    let unresolved = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    let approximate =
        hyperlimit::classify_real_sign(&unresolved, hyperlimit::PredicatePolicy::APPROXIMATE_512);
    assert!(matches!(
        approximate,
        PredicateOutcome::Decided {
            certainty: Certainty::Approximate,
            ..
        }
    ));
    assert!(matches!(
        force().dynamics(unresolved, &mut budget()),
        Err(DynamicsError::MassSign(PredicateOutcome::Unknown { .. }))
    ));
}

#[test]
fn rejected_evolution_never_publishes_a_completed_endpoint() {
    let model = force().dynamics(Real::from(2), &mut budget()).unwrap();
    let request = EvolutionRequest {
        start: point(0),
        end: point(1),
        initial: zero(),
    };
    let stopped = model.integrate(
        request,
        EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            maximum_steps: 1,
            ..EvolutionOptions::default()
        },
        &mut budget(),
    );
    assert!(matches!(
        stopped.native().status(),
        EvolutionStatus::Stopped(_)
    ));
    assert!(stopped.native().end_enclosure().is_none());
    assert!(stopped.native().prefix_end_enclosure().is_some());
    assert_eq!(stopped.native().reached_time(), &d(1, 1, -2));
    assert_eq!(stopped.model().mass(), &Real::from(2));
    let rejected = model.certify_step(
        proposal(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            scalar_work: 0,
            ..EnclosurePolicy::default()
        }),
    );
    assert!(matches!(
        rejected.native().status(),
        PicardStatus::Rejected(_)
    ));
    assert!(rejected.native().end_enclosure().is_none());
}

#[test]
fn exact_source_replay_rebuilds_physics_and_native_proofs_in_a_fresh_process() {
    const KEY: &str = "HYPERPHYSICS_FLOW_REPLAY_CHILD";
    if let Some(payload) = std::env::var_os(KEY) {
        let values: Vec<i64> = payload
            .to_str()
            .unwrap()
            .split(',')
            .map(|v| v.parse().unwrap())
            .collect();
        assert_eq!(values.len(), 4);
        let mut forces = ForceAccumulator3::default();
        forces.push(ForceContribution3 {
            source: "fixture wire definition".into(),
            force: Vector3::new(std::array::from_fn(|i| Real::from(values[i]))),
        });
        let model = forces
            .dynamics(Real::from(values[3]), &mut budget())
            .unwrap();
        let report = model.certify_step(proposal(), &mut budget());
        assert!(matches!(report.native().status(), PicardStatus::Certified));
        let end = report.native().end_enclosure().unwrap().lanes().unwrap();
        assert_eq!(end[3], d(1, 1, -1));
        assert_eq!(end[4], d(-1, -1, -1));
        assert_contains(&end[0], 1, -4);
        assert_contains(&end[1], -1, -4);
        assert_eq!(model.provenance()[0].locator, "fixture wire definition");
        return;
    }
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "exact_source_replay_rebuilds_physics_and_native_proofs_in_a_fresh_process",
            "--nocapture",
        ])
        .env(KEY, "4,-4,0,2")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn retained_physical_and_native_getters_do_not_recompute_predicates() {
    let model = force().dynamics(Real::from(2), &mut budget()).unwrap();
    let report = model.certify_step(proposal(), &mut budget());
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    std::hint::black_box(report.model().mass());
    std::hint::black_box(report.model().mass_sign());
    std::hint::black_box(report.model().force_source());
    std::hint::black_box(report.model().provenance());
    std::hint::black_box(report.model().constant_forces());
    std::hint::black_box(report.model().field());
    std::hint::black_box(report.native().status());
    std::hint::black_box(report.native().end_enclosure());
    std::hint::black_box(report.native().tube());
    assert!(hyperreal::dispatch_trace::snapshot().is_empty());
}
