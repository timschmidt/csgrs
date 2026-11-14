#![cfg(feature = "reachability")]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, FieldError, evolution::*,
    reachability::*,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn sb() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn field<const N: usize>(outputs: &[Expr]) -> ExpressionField<N> {
    let state: Vec<_> = (0..N).map(|i| SymbolId(i as u32)).collect();
    let time = SymbolId(100);
    let mut vars = state.clone();
    vars.push(time);
    let source =
        ExpressionProgram::compile(outputs, &vars, &mut ExpressionBudget::new(100000, 10000000))
            .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state,
        },
        &mut b(),
    )
    .unwrap()
}
fn request<const N: usize>(
    initial: BoxN<DyadicInterval, N>,
    epsilon: DyadicInterval,
) -> ReachabilityRequest<N> {
    ReachabilityRequest {
        evolution: EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial,
        },
        epsilon,
    }
}
fn check<const N: usize>(r: &ReachabilityReport<N>) {
    assert!(
        matches!(r.status(), ReachabilityStatus::Completed),
        "{:?}",
        r.status()
    );
    assert!(r.end_cover().is_some());
    for (i, cell) in r.cells().iter().enumerate() {
        if let Some([left, right]) = cell.children() {
            assert_eq!(r.cells()[left].parent(), Some(i));
            assert_eq!(r.cells()[right].parent(), Some(i));
            assert_eq!(r.cells()[left].depth(), cell.depth() + 1);
            assert_eq!(r.cells()[right].depth(), cell.depth() + 1);
            let axis = cell.split_axis().unwrap();
            let source = cell.initial().lanes().unwrap();
            let l = r.cells()[left].initial().lanes().unwrap();
            let u = r.cells()[right].initial().lanes().unwrap();
            for k in 0..N {
                if k == axis {
                    let s = source[k].to_rational(&mut sb()).unwrap();
                    let a = l[k].to_rational(&mut sb()).unwrap();
                    let z = u[k].to_rational(&mut sb()).unwrap();
                    let midpoint = (s.lower().unwrap().clone() + s.upper().unwrap().clone())
                        / hyperreal::Rational::from(2);
                    assert_eq!(a.lower(), s.lower());
                    assert_eq!(z.upper(), s.upper());
                    assert_eq!(a.upper(), Some(&midpoint));
                    assert_eq!(z.lower(), Some(&midpoint));
                } else {
                    assert_eq!(l[k], source[k]);
                    assert_eq!(u[k], source[k]);
                }
            }
        }
    }
    for a in r.attempts() {
        assert_eq!(a.evolution().identity(), r.identity());
        assert_eq!(
            &a.evolution().request().initial,
            r.cells()[a.cell()].initial()
        );
        assert_eq!(a.evolution().request().start, r.request().evolution.start);
        assert_eq!(a.evolution().request().end, r.request().evolution.end);
        assert_eq!(a.evolution().policy(), r.enclosure_policy());
    }
    for endpoint in r.end_cover().unwrap().boxes() {
        for lane in endpoint.lanes().unwrap() {
            assert!(
                lane.width(&mut sb())
                    .unwrap()
                    .compare_singleton(&r.request().epsilon, &mut sb())
                    .unwrap()
                    != core::cmp::Ordering::Greater
            );
        }
    }
}
#[test]
fn exact_constant_cover_final_allowed_contribution_and_composition() {
    let f = field::<1>(&[Expr::int(2)]);
    let req = request(BoxN::new([d(-2, 2, 0)]).unwrap(), d(1, 1, 0));
    let options = ReachabilityPolicy {
        maximum_cells: 7,
        maximum_evolutions: 7,
        maximum_depth: 2,
        ..Default::default()
    };
    let r = f.reach(req.clone(), options.clone(), &mut b());
    check(&r);
    assert_eq!(r.cells().len(), 7);
    assert_eq!(r.attempts().len(), 7);
    let cover = r.end_cover().unwrap();
    let boxes: Vec<_> = cover.boxes().cloned().collect();
    assert_eq!(
        boxes,
        (0..4)
            .map(|i| BoxN::new([d(i - 1, i, 0)]).unwrap())
            .collect::<Vec<_>>()
    );
    for (i, endpoint) in boxes.into_iter().enumerate() {
        let mut next = request(endpoint, d(1, 1, 0));
        next.evolution.start = d(1, 1, -1);
        next.evolution.end = d(1, 1, 0);
        let r = f.reach(next, Default::default(), &mut b());
        check(&r);
        assert_eq!(
            r.end_cover().unwrap().boxes().next().unwrap(),
            &BoxN::new([d(i as i64, i as i64 + 1, 0)]).unwrap()
        );
    }
    let fail = f.reach(
        req,
        ReachabilityPolicy {
            maximum_evolutions: 6,
            ..options
        },
        &mut b(),
    );
    assert!(matches!(
        fail.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::EvolutionLimit { limit: 6 })
    ));
    assert!(fail.end_cover().is_none());
    assert_eq!(fail.attempts().len(), 6);
}
#[test]
fn exact_partition_axis_ties_and_deterministic_source_replay() {
    let f = field::<2>(&[Expr::int(0), Expr::int(0)]);
    let req = request(BoxN::new([d(0, 2, 0), d(-1, 1, 0)]).unwrap(), d(1, 1, 0));
    let r = f.reach(req.clone(), Default::default(), &mut b());
    check(&r);
    assert_eq!(r.cells()[0].split_axis(), Some(0));
    let rebuilt = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: r.identity().source().clone(),
            time: r.identity().time(),
            state: r.identity().state().to_vec(),
        },
        &mut b(),
    )
    .unwrap();
    let replay = rebuilt.reach(req, Default::default(), &mut b());
    assert_eq!(format!("{r:?}"), format!("{replay:?}"));
    assert_eq!(r.end_cover().unwrap().boxes().len(), 4);
}
#[test]
fn empty_unit_identity_and_invalid_input_precedence() {
    let f = field::<1>(&[Expr::int(0)]);
    let empty = request(BoxN::empty().unwrap(), d(0, 0, 0));
    let r = f.reach(empty.clone(), Default::default(), &mut b());
    assert!(matches!(r.status(), ReachabilityStatus::Empty));
    assert_eq!(r.end_cover().unwrap().boxes().len(), 0);
    for epsilon in [DyadicInterval::empty(), d(-1, -1, 0), d(0, 1, 0)] {
        let r = f.reach(
            ReachabilityRequest {
                epsilon,
                ..empty.clone()
            },
            Default::default(),
            &mut b(),
        );
        assert!(matches!(
            r.status(),
            ReachabilityStatus::Stopped(ReachabilityFailure::InvalidEpsilon)
        ));
        assert!(r.end_cover().is_none());
    }
    let mut invalid = empty.clone();
    invalid.evolution.start = d(0, 1, 0);
    let r = f.reach(invalid, Default::default(), &mut b());
    assert!(matches!(
        r.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::Evolution {
            reason: EvolutionFailure::NonPointTime,
            ..
        })
    ));
    let mut invalid = empty;
    invalid.evolution.start = d(1, 1, 0);
    let r = f.reach(invalid, Default::default(), &mut b());
    assert!(matches!(
        r.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::Evolution {
            reason: EvolutionFailure::ReversedTime,
            ..
        })
    ));
    let initial = BoxN::new([d(-10, 10, 0)]).unwrap();
    let mut identity = request(initial.clone(), d(0, 0, 0));
    identity.evolution.end = d(0, 0, 0);
    let r = f.reach(identity, Default::default(), &mut b());
    assert!(matches!(r.status(), ReachabilityStatus::Identity));
    assert_eq!(r.end_cover().unwrap().boxes().next(), Some(&initial));
    let unit = field::<0>(&[]).reach(
        request(BoxN::new([]).unwrap(), d(0, 0, 0)),
        Default::default(),
        &mut b(),
    );
    check(&unit);
    assert_eq!(unit.end_cover().unwrap().boxes().len(), 1);
}
#[test]
fn limits_cancellation_exhaustion_and_retry_remain_typed() {
    let f = field::<1>(&[Expr::int(0)]);
    let req = request(BoxN::new([d(0, 4, 0)]).unwrap(), d(1, 1, 0));
    let r = f.reach(
        req.clone(),
        ReachabilityPolicy {
            maximum_cells: 2,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        r.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::CellCapacity {
            limit: 2,
            required: 3
        })
    ));
    let r = f.reach(
        req.clone(),
        ReachabilityPolicy {
            maximum_depth: 0,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        r.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::DepthLimit { cell: 0, limit: 0 })
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut budget = b();
    budget.set_cancellation(Some(token));
    let r = f.reach(req.clone(), Default::default(), &mut budget);
    assert!(matches!(
        r.status(),
        ReachabilityStatus::Stopped(ReachabilityFailure::Preparation(EvolutionFailure::Field(
            FieldError::Expression(ExpressionError::Cancelled)
        )))
    ));
    let mut budget = EnclosureBudget::new(EnclosurePolicy {
        graph_work: 0,
        ..Default::default()
    });
    let r = f.reach(req.clone(), Default::default(), &mut budget);
    assert!(matches!(r.status(), ReachabilityStatus::Stopped(_)));
    assert!(r.end_cover().is_none());
    check(&f.reach(req, Default::default(), &mut b()));
}
#[test]
fn affine_point_accuracy_refines_time_and_keeps_every_evolution() {
    let f = field::<1>(&[Expr::anonymous_symbol(SymbolId(0))]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap(), d(1, 1, -3));
    let r = f.reach(req, Default::default(), &mut b());
    check(&r);
    assert!(r.attempts().len() > 1);
    assert_eq!(r.cells().len(), 1);
    assert!(r.cells()[0].time_refinements() > 0);
    let endpoint = r
        .end_cover()
        .unwrap()
        .boxes()
        .next()
        .unwrap()
        .lanes()
        .unwrap()[0]
        .to_rational(&mut sb())
        .unwrap();
    // Independent positive Taylor sum and geometric tail bound for exp(1/2).
    let mut term = hyperreal::Rational::from(1);
    let mut sum = term.clone();
    for k in 1..=32 {
        term = term / hyperreal::Rational::from(2 * k);
        sum = sum + term.clone();
    }
    let upper = sum.clone()
        + term / hyperreal::Rational::from(66) * hyperreal::Rational::fraction(68, 67).unwrap();
    assert!(endpoint.lower().unwrap() <= &sum);
    assert!(endpoint.upper().unwrap() >= &upper);
    for pair in r.attempts().windows(2) {
        let old = pair[0]
            .evolution()
            .segments()
            .next()
            .unwrap()
            .evidence()
            .duration()
            .unwrap();
        let new = pair[1].evolution().options().maximum_step.as_ref().unwrap();
        assert!(new.compare_singleton(old, &mut sb()).unwrap() == core::cmp::Ordering::Less);
    }
}

#[test]
fn nonlinear_uncertain_cover_refines_time_and_encloses_exact_fiber_images() {
    let x = Expr::anonymous_symbol(SymbolId(0));
    let f = field::<1>(&[x.clone() * x]);
    let req = request(BoxN::new([d(1, 2, -2)]).unwrap(), d(1, 1, -7));
    let policy = ReachabilityPolicy {
        maximum_depth: 8,
        maximum_evolutions: 2048,
        ..Default::default()
    };
    let r = f.reach(req, policy, &mut b());
    if r.end_cover().is_none() {
        for (i, a) in r.attempts().iter().enumerate() {
            eprintln!(
                "attempt {i} cell {} steps {} cap {:?} status {:?}",
                a.cell(),
                a.evolution().steps().len(),
                a.evolution().options().maximum_step,
                a.evolution().status()
            );
        }
    }
    check(&r);
    assert!(r.cells().iter().any(|c| c.time_refinements() > 0));
    let epsilon = hyperreal::Rational::fraction(1, 128).unwrap();
    for cell in r.cells().iter().filter(|c| c.children().is_none()) {
        let initial = cell.initial().lanes().unwrap()[0]
            .to_rational(&mut sb())
            .unwrap();
        let endpoint = r.attempts()[cell.last_attempt().unwrap()]
            .evolution()
            .end_enclosure()
            .unwrap()
            .lanes()
            .unwrap()[0]
            .to_rational(&mut sb())
            .unwrap();
        let image = |x: &hyperreal::Rational| {
            x.clone() / (hyperreal::Rational::from(1) - x.clone() / hyperreal::Rational::from(2))
        };
        let lower = image(initial.lower().unwrap());
        let upper = image(initial.upper().unwrap());
        assert!(endpoint.lower().unwrap() <= &lower);
        assert!(endpoint.upper().unwrap() >= &upper);
        assert!(endpoint.lower().unwrap() >= &(lower - epsilon.clone()));
        assert!(endpoint.upper().unwrap() <= &(upper + epsilon.clone()));
    }
}

#[test]
fn every_structural_cutoff_and_final_scalar_boundary_withhold_partial_cover() {
    let f = field::<1>(&[Expr::int(2)]);
    let req = request(BoxN::new([d(0, 2, 0)]).unwrap(), d(1, 1, 0));
    let policy = ReachabilityPolicy {
        evolution: EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        },
        ..Default::default()
    };
    let mut full = b();
    let report = f.reach(req.clone(), policy.clone(), &mut full);
    check(&report);
    let work = full.policy().graph_work - full.remaining_graph_work();
    let scalar = full.policy().scalar_work - full.remaining_scalar_work();
    for limit in 0..=work {
        let mut budget = EnclosureBudget::new(EnclosurePolicy {
            graph_work: limit,
            ..Default::default()
        });
        let r = f.reach(req.clone(), policy.clone(), &mut budget);
        assert_eq!(
            r.end_cover().is_some(),
            limit == work,
            "graph limit {limit}, {:?}",
            r.status()
        );
        if limit < work {
            assert!(matches!(r.status(), ReachabilityStatus::Stopped(_)));
        }
    }
    for limit in [0, scalar / 2, scalar - 1, scalar] {
        let mut budget = EnclosureBudget::new(EnclosurePolicy {
            scalar_work: limit,
            ..Default::default()
        });
        let r = f.reach(req.clone(), policy.clone(), &mut budget);
        assert_eq!(
            r.end_cover().is_some(),
            limit == scalar,
            "scalar limit {limit}, {:?}",
            r.status()
        );
    }
    check(&f.reach(req, policy, &mut b()));
}

#[test]
fn prefix_transport_is_explicit_connected_and_does_not_relabel_native_limits() {
    let f = field::<1>(&[Expr::anonymous_symbol(SymbolId(0))]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap(), d(1, 1, -3));
    let r = f.reach(req, Default::default(), &mut b());
    check(&r);
    let mut saw_partial = false;
    let mut saw_rounding = false;
    let mut saw_displacement = false;
    for attempt in r.attempts() {
        let evolution = attempt.evolution();
        let mut time = &evolution.request().start;
        let mut state = &evolution.request().initial;
        for step in evolution.steps() {
            assert_eq!(&step.native().request().start, time);
            assert_eq!(&step.native().request().initial, state);
            // Reconstruct proposal provenance with exact rational endpoints,
            // independently of the interval padding implementation.
            for (index, search) in step.native().attempts().iter().enumerate() {
                if let CandidateSource::PicardDisplacementExpansion { attempt } = search.source() {
                    saw_displacement = true;
                    assert_eq!(attempt + 1, index);
                    let previous = &step.native().attempts()[attempt];
                    assert_eq!(previous.segment(), search.segment());
                    assert_eq!(previous.halvings(), search.halvings());
                    assert_eq!(previous.expansions() + 1, search.expansions());
                    let old = previous.report();
                    assert!(matches!(
                        old.status(),
                        hyperdynamics::field::picard::PicardStatus::Rejected(
                            hyperdynamics::field::picard::PicardFailure::SelfMapNotCertified
                        )
                    ));
                    let initial = old.proposal().initial.lanes().unwrap()[0]
                        .to_rational(&mut sb())
                        .unwrap();
                    let candidate = old.proposal().candidate.lanes().unwrap()[0]
                        .to_rational(&mut sb())
                        .unwrap();
                    let image = old.evidence().picard_image().unwrap().lanes().unwrap()[0]
                        .to_rational(&mut sb())
                        .unwrap();
                    let lower = if candidate.lower().unwrap() < image.lower().unwrap() {
                        candidate.lower().unwrap()
                    } else {
                        image.lower().unwrap()
                    };
                    let upper = if candidate.upper().unwrap() > image.upper().unwrap() {
                        candidate.upper().unwrap()
                    } else {
                        image.upper().unwrap()
                    };
                    let growth = upper.clone()
                        - lower.clone()
                        - (initial.upper().unwrap().clone() - initial.lower().unwrap().clone());
                    assert!(growth >= hyperreal::Rational::from(0));
                    let next = search.report().proposal().candidate.lanes().unwrap()[0]
                        .to_rational(&mut sb())
                        .unwrap();
                    assert_eq!(next.lower().unwrap(), &(lower.clone() - growth.clone()));
                    assert_eq!(next.upper().unwrap(), &(upper.clone() + growth));
                }
            }
            if let Some(carried) = step.carried() {
                let proof = step.native().segment(0).unwrap();
                assert!(
                    proof
                        .end_enclosure()
                        .unwrap()
                        .subset_of(carried, &mut sb())
                        .unwrap()
                );
                saw_rounding |= proof.end_enclosure().unwrap() != carried;
                if matches!(
                    step.native().status(),
                    EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 })
                ) {
                    saw_partial = true;
                    assert!(step.native().end_enclosure().is_none());
                }
                time = &proof.proposal().end;
                state = carried;
            }
        }
        if evolution.end_enclosure().is_some() {
            assert_eq!(time, &evolution.request().end);
            assert_eq!(Some(state), evolution.end_enclosure());
        }
    }
    assert!(saw_partial);
    assert!(saw_rounding);
    assert!(saw_displacement);
}

#[test]
fn domain_failure_and_coarse_grid_never_become_an_accuracy_success() {
    let x = Expr::anonymous_symbol(SymbolId(0));
    let f = field::<1>(&[Expr::int(1) / x]);
    let req = request(BoxN::new([d(-1, 1, 0)]).unwrap(), d(1, 1, -3));
    let r = f.reach(
        req,
        ReachabilityPolicy {
            maximum_depth: 2,
            maximum_evolutions: 32,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(r.status(), ReachabilityStatus::Stopped(_)));
    assert!(r.end_cover().is_none());
    assert!(
        r.attempts()
            .iter()
            .any(|a| matches!(a.evolution().status(), EvolutionStatus::Stopped(_)))
    );
    let f = field::<1>(&[Expr::anonymous_symbol(SymbolId(0))]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap(), d(1, 1, -7));
    let mut budget = EnclosureBudget::new(EnclosurePolicy {
        precision: -2,
        ..Default::default()
    });
    let r = f.reach(
        req,
        ReachabilityPolicy {
            maximum_time_refinements: 2,
            ..Default::default()
        },
        &mut budget,
    );
    assert!(matches!(r.status(), ReachabilityStatus::Stopped(_)));
    assert!(r.end_cover().is_none());
}

#[cfg(feature = "serde")]
#[test]
fn source_request_and_policy_replay_in_fresh_processes() {
    const KEY: &str = "HYPERDYNAMICS_REACHABILITY_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (source, request, policy): (
            FieldDefinition,
            ReachabilityRequest<2>,
            ReachabilityPolicy,
        ) = serde_json::from_str(&encoded).unwrap();
        let mut budget = b();
        let field = ExpressionField::<2>::from_definition(source, &mut budget).unwrap();
        let result = field.reach(request, policy, &mut budget);
        check(&result);
        println!(
            "REPLAY|{result:?}|{}|{}",
            budget.remaining_graph_work(),
            budget.remaining_scalar_work()
        );
        return;
    }
    let f = field::<2>(&[Expr::int(2), Expr::int(-1)]);
    let req = request(BoxN::new([d(-1, 1, 0), d(0, 2, 0)]).unwrap(), d(1, 1, 0));
    let r = f.reach(req.clone(), Default::default(), &mut b());
    let encoded =
        serde_json::to_string(&(r.identity(), req, ReachabilityPolicy::default())).unwrap();
    let mut expected = None;
    for _ in 0..2 {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "source_request_and_policy_replay_in_fresh_processes",
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
        let row = text
            .lines()
            .find(|s| s.starts_with("REPLAY|"))
            .unwrap()
            .to_string();
        if let Some(old) = expected.as_ref() {
            assert_eq!(old, &row)
        } else {
            expected = Some(row)
        }
    }
    let mut bad: serde_json::Value = serde_json::from_str(&encoded).unwrap();
    bad[0]["state"] = serde_json::json!([0, 0]);
    let (source, _, _): (FieldDefinition, ReachabilityRequest<2>, ReachabilityPolicy) =
        serde_json::from_value(bad).unwrap();
    assert!(ExpressionField::<2>::from_definition(source, &mut b()).is_err());
}

#[test]
fn regridded_exponential_completes_the_growth_case_under_default_resources() {
    use hyperinterval::IntervalError;
    use hyperlattice_interval::IntervalLatticeError;
    let f = field::<1>(&[Expr::anonymous_symbol(SymbolId(0))]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap(), d(1, 1, -9));
    let options = EvolutionOptions {
        maximum_step: Some(d(1, 1, -12)),
        maximum_steps: 16384,
        ..Default::default()
    };
    // Preserve the original hard case: ordinary unrounded continuation reaches
    // the real scalar-size bound, not a mathematical no-solution result.
    let ordinary = f.integrate(req.evolution.clone(), options.clone(), &mut b());
    assert!(
        matches!(
            ordinary.status(),
            EvolutionStatus::Stopped(EvolutionFailure::Step(
                hyperdynamics::field::picard::PicardFailure::Field(FieldError::Lattice(
                    IntervalLatticeError::Scalar(IntervalError::SizeExhausted)
                ))
            ))
        ),
        "{:?}",
        ordinary.status()
    );
    assert!(ordinary.end_enclosure().is_none());
    assert!(
        ordinary
            .reached_time()
            .compare_singleton(&req.evolution.end, &mut sb())
            .unwrap()
            .is_lt()
    );
    let policy = ReachabilityPolicy {
        evolution: options,
        maximum_cells: 1,
        maximum_evolutions: 1,
        maximum_depth: 0,
        maximum_time_refinements: 0,
    };
    let mut work = b();
    let r = f.reach(req.clone(), policy.clone(), &mut work);
    check(&r);
    assert_eq!(r.cells().len(), 1);
    assert_eq!(r.attempts().len(), 1);
    let evolution = r.attempts()[0].evolution();
    assert_eq!(evolution.steps().len(), 2048);
    let mut time = &req.evolution.start;
    let mut state = &req.evolution.initial;
    let mut rounded = 0;
    for step in evolution.steps() {
        let proof = step.native().segment(0).unwrap();
        assert_eq!(&proof.proposal().start, time);
        assert_eq!(&proof.proposal().initial, state);
        let carried = step.carried().unwrap();
        assert!(
            proof
                .end_enclosure()
                .unwrap()
                .subset_of(carried, &mut sb())
                .unwrap()
        );
        assert!(carried.lanes().unwrap()[0].bounds().unwrap().2 >= -64);
        rounded += usize::from(proof.end_enclosure().unwrap() != carried);
        time = &proof.proposal().end;
        state = carried;
    }
    assert!(rounded > 0);
    assert_eq!(time, &req.evolution.end);
    let cover = r.end_cover().unwrap();
    assert_eq!(cover.boxes().len(), 1);
    assert_eq!(cover.end_time(), &req.evolution.end);
    assert_eq!(cover.epsilon(), &req.epsilon);
    let endpoint = cover.boxes().next().unwrap();
    assert_eq!(endpoint, state);
    let value = endpoint.lanes().unwrap()[0].to_rational(&mut sb()).unwrap();
    let mut term = hyperreal::Rational::from(1);
    let mut sum = term.clone();
    for k in 1..=32 {
        term = term / hyperreal::Rational::from(2 * k);
        sum = sum + &term;
    }
    let upper = &sum
        + term / hyperreal::Rational::from(66) * hyperreal::Rational::fraction(68, 67).unwrap();
    assert!(value.contains_point(&sum));
    assert!(value.contains_point(&upper));
    let spent_graph = work.policy().graph_work - work.remaining_graph_work();
    let spent_scalar = work.policy().scalar_work - work.remaining_scalar_work();
    let mut larger = EnclosureBudget::new(EnclosurePolicy {
        scalar_bits: 1_048_576,
        scalar_work: 100_000_000_000_000,
        ..EnclosurePolicy::default()
    });
    let again = f.reach(req, policy, &mut larger);
    check(&again);
    assert_eq!(again.end_cover().unwrap().boxes().next(), Some(endpoint));
    assert_eq!(
        larger.policy().graph_work - larger.remaining_graph_work(),
        spent_graph
    );
    assert_eq!(
        larger.policy().scalar_work - larger.remaining_scalar_work(),
        spent_scalar
    );

    #[cfg(feature = "taylor-evolution")]
    {
        use hyperdynamics::field::taylor_flow::evolution::FlowTaylorEvolutionStatus;
        let p = f.flow_taylor(3, &mut b()).unwrap();
        let next = p.integrate(
            EvolutionRequest {
                start: d(1, 1, -1),
                end: d(3, 3, -2),
                initial: endpoint.clone(),
            },
            EvolutionOptions {
                maximum_step: Some(d(1, 1, -2)),
                ..Default::default()
            },
            &mut b(),
        );
        assert!(
            matches!(next.status(), FlowTaylorEvolutionStatus::Completed),
            "{:?}",
            next.status()
        );
        let end = next.end_enclosure().unwrap().lanes().unwrap()[0]
            .to_rational(&mut sb())
            .unwrap();
        let mut term = hyperreal::Rational::from(1);
        let mut lower = term.clone();
        for k in 1..=16 {
            term *= hyperreal::Rational::fraction(3, 4 * k).unwrap();
            lower = lower + &term;
        }
        let upper = &lower
            + term
                * hyperreal::Rational::fraction(3, 68).unwrap()
                * hyperreal::Rational::fraction(24, 23).unwrap();
        assert!(end.contains_point(&lower));
        assert!(end.contains_point(&upper));
    }
}

#[test]
fn regridded_portfolio_cap_meets_quality_while_the_coarser_control_does_not() {
    let f = field::<1>(&[Expr::anonymous_symbol(SymbolId(0))]);
    let req = request(BoxN::new([d(1, 1, 0)]).unwrap(), d(1, 1, -9));
    for (exponent, count, completes) in [(-10, 512, false), (-11, 1024, true)] {
        let r = f.reach(
            req.clone(),
            ReachabilityPolicy {
                evolution: EvolutionOptions {
                    maximum_step: Some(d(1, 1, exponent)),
                    maximum_steps: 16384,
                    ..Default::default()
                },
                maximum_cells: 1,
                maximum_evolutions: 1,
                maximum_depth: 0,
                maximum_time_refinements: 0,
            },
            &mut b(),
        );
        assert_eq!(r.cells().len(), 1);
        assert_eq!(r.attempts().len(), 1);
        assert_eq!(r.attempts()[0].evolution().steps().len(), count);
        assert!(matches!(
            r.attempts()[0].evolution().status(),
            EvolutionStatus::Completed
        ));
        if completes {
            check(&r);
            let endpoint = r
                .end_cover()
                .unwrap()
                .boxes()
                .next()
                .unwrap()
                .lanes()
                .unwrap()[0]
                .to_rational(&mut sb())
                .unwrap();
            let mut term = hyperreal::Rational::from(1);
            let mut lower = term.clone();
            for k in 1..=32 {
                term = term / hyperreal::Rational::from(2 * k);
                lower = lower + &term;
            }
            let upper = &lower
                + term / hyperreal::Rational::from(66)
                    * hyperreal::Rational::fraction(68, 67).unwrap();
            assert!(endpoint.contains_point(&lower));
            assert!(endpoint.contains_point(&upper));
        } else {
            assert!(matches!(
                r.status(),
                ReachabilityStatus::Stopped(ReachabilityFailure::TimeRefinementLimit {
                    cell: 0,
                    limit: 0
                })
            ));
            assert!(r.end_cover().is_none());
            assert!(
                r.cells()[0]
                    .end_diameter()
                    .unwrap()
                    .compare_singleton(&req.epsilon, &mut sb())
                    .unwrap()
                    .is_gt()
            );
        }
    }
}
