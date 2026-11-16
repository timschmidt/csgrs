#![cfg(feature = "taylor-flow")]

use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, picard::PicardProposal,
    taylor_flow::FlowTaylorStatus,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

#[cfg(feature = "flow-noise")]
#[path = "../examples/flow_noise.rs"]
mod shipped_example;

#[cfg(feature = "flow-noise")]
#[path = "support/flow_oracles.rs"]
mod flow_oracles;

#[test]
#[cfg(feature = "flow-noise")]
fn shipped_example_retains_uncertainty_through_both_steps() {
    shipped_example::run().unwrap();
}

fn interval(lower: i64, upper: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lower.into(), upper.into(), exponent).unwrap()
}

fn shear(sign: i64, budget: &mut EnclosureBudget) -> ExpressionField<2> {
    let time = SymbolId(10);
    let x = SymbolId(20);
    let y = SymbolId(30);
    let source = ExpressionProgram::compile(
        &[Expr::int(sign) * Expr::anonymous_symbol(y), Expr::int(0)],
        &[y, time, x],
        &mut ExpressionBudget::new(100_000, 1_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x, y],
        },
        budget,
    )
    .unwrap()
}

#[test]
fn cartesian_endpoints_lose_reversible_shear_correlation() {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let initial = BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap();
    let mut state = initial.clone();
    for (step, sign) in [(0, 1), (1, -1)] {
        let field = shear(sign, &mut budget);
        let report = field.flow_taylor(1, &mut budget).unwrap().certify_step(
            PicardProposal {
                start: interval(step, step, -2),
                end: interval(step + 1, step + 1, -2),
                initial: state,
                candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
            },
            &mut budget,
        );
        assert!(matches!(report.status(), FlowTaylorStatus::Certified));
        state = report.end_enclosure().unwrap().clone();
    }
    // Exact composed map: (x + y/4) - y/4 = x, with y unchanged.
    // The existing Cartesian composition instead permits independent copies of y.
    assert_eq!(
        state.lanes(),
        Some(&[interval(-3, 3, -1), interval(-1, 1, 0)])
    );
    assert_ne!(state, initial);
    println!("exact identity endpoint: {initial:?}; Cartesian endpoint: {state:?}");
}

#[test]
#[cfg(feature = "flow-noise")]
fn retained_columns_recover_exact_identity_over_repeated_shear_pairs() {
    use hyperdynamics::field::taylor_flow::noise::{
        FlowNoiseRequest, FlowNoiseState, FlowNoiseStatus,
    };
    use std::sync::Arc;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let initial = BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap();
    let forward = shear(1, &mut budget).flow_taylor(1, &mut budget).unwrap();
    let reverse = shear(-1, &mut budget).flow_taylor(1, &mut budget).unwrap();
    let mut state =
        Arc::new(FlowNoiseState::new(interval(0, 0, 0), initial.clone(), &mut budget).unwrap());
    for step in 0..6 {
        let program = if step % 2 == 0 { &forward } else { &reverse };
        let old = state.clone();
        let report = program.certify_noise_step(
            state,
            FlowNoiseRequest {
                end: interval(step + 1, step + 1, -2),
                candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
                maximum_noise: 2,
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseStatus::Certified),
            "{:?}",
            report.status()
        );
        state = report.end_state().unwrap().clone();
        assert!(Arc::ptr_eq(report.evidence().initial(), &old));
        assert!(Arc::ptr_eq(state.evidence().unwrap(), report.evidence()));
        assert_eq!(
            state.residual(),
            Some(&[interval(0, 0, 0), interval(0, 0, 0)])
        );
        assert_eq!(state.time(), &interval(step + 1, step + 1, -2));
        if step % 2 == 1 {
            assert_eq!(state.enclosure(), &initial);
        } else {
            assert_eq!(
                state.enclosure().lanes(),
                Some(&[interval(-5, 5, -2), interval(-1, 1, 0)])
            );
        }
    }
}

#[cfg(feature = "flow-noise")]
mod closure {
    use super::*;
    use hyperdynamics::field::{
        FieldError,
        taylor_flow::noise::{
            FlowNoiseError, FlowNoiseInput, FlowNoiseRequest, FlowNoiseState, FlowNoiseStatus,
        },
    };
    use hypersolve::expression_program::ExpressionError;
    use std::sync::Arc;

    fn budget() -> EnclosureBudget {
        EnclosureBudget::new(EnclosurePolicy::default())
    }

    #[test]
    fn algebraic_shear_coefficients_cancel_without_scalar_demotion() {
        let time = SymbolId(10);
        let x = SymbolId(20);
        let y = SymbolId(30);
        let coefficient = hyperreal::Real::from(2).sqrt().unwrap();
        let mut budget = budget();
        let initial = BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap();
        let mut state =
            Arc::new(FlowNoiseState::new(interval(0, 0, 0), initial.clone(), &mut budget).unwrap());
        for (step, sign) in [(0, 1), (1, -1)] {
            let source = ExpressionProgram::compile(
                &[
                    Expr::int(sign) * Expr::real(coefficient.clone()) * Expr::anonymous_symbol(y),
                    Expr::int(0),
                ],
                &[y, time, x],
                &mut ExpressionBudget::new(100_000, 1_000_000),
            )
            .unwrap();
            let field = ExpressionField::<2>::from_definition(
                FieldDefinition {
                    source: source.definition().clone(),
                    time,
                    state: vec![x, y],
                },
                &mut budget,
            )
            .unwrap();
            let report = field
                .flow_taylor(1, &mut budget)
                .unwrap()
                .certify_noise_step(
                    state,
                    FlowNoiseRequest {
                        end: interval(step + 1, step + 1, -2),
                        candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
                        maximum_noise: 2,
                    },
                    &mut budget,
                );
            assert!(
                matches!(report.status(), FlowNoiseStatus::Certified),
                "{:?}",
                report.status()
            );
            state = report.end_state().unwrap().clone();
        }
        // (x + sqrt(2)y/4) - sqrt(2)y/4 = x for every point of the box.
        assert_eq!(state.enclosure(), &initial);
        assert_eq!(
            state.residual(),
            Some(&[interval(0, 0, 0), interval(0, 0, 0)])
        );
    }

    #[test]
    fn zero_capacity_charges_all_columns_to_the_residual() {
        let mut budget = budget();
        let mut state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap(),
                &mut budget,
            )
            .unwrap(),
        );
        for (step, sign) in [(0, 1), (1, -1)] {
            let report = shear(sign, &mut budget)
                .flow_taylor(1, &mut budget)
                .unwrap()
                .certify_noise_step(
                    state,
                    FlowNoiseRequest {
                        end: interval(step + 1, step + 1, -2),
                        candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
                        maximum_noise: 0,
                    },
                    &mut budget,
                );
            assert!(
                matches!(report.status(), FlowNoiseStatus::Certified),
                "{:?}",
                report.status()
            );
            assert!(report.evidence().noise().unwrap().columns().is_empty());
            assert!(
                !report
                    .evidence()
                    .noise()
                    .unwrap()
                    .discarded_terms()
                    .is_empty()
            );
            if step == 1 {
                assert_eq!(
                    report.evidence().inputs(),
                    &[FlowNoiseInput::Residual(0), FlowNoiseInput::Residual(1)]
                );
            }
            state = report.end_state().unwrap().clone();
            assert!(state.columns().unwrap().is_empty());
        }
        assert_eq!(
            state.enclosure().lanes(),
            Some(&[interval(-3, 3, -1), interval(-1, 1, 0)])
        );
    }

    #[test]
    fn nonlinear_residual_is_fresh_and_riccati_endpoint_encloses_exact_solutions() {
        use hyperinterval::DyadicBudget;
        use hyperreal::Rational;
        let time = SymbolId(10);
        let x = SymbolId(20);
        let expression = Expr::anonymous_symbol(x);
        let source = ExpressionProgram::compile(
            &[expression.clone() * expression],
            &[x, time],
            &mut ExpressionBudget::new(100_000, 1_000_000),
        )
        .unwrap();
        let mut budget = budget();
        let field = ExpressionField::<1>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x],
            },
            &mut budget,
        )
        .unwrap();
        let program = field.flow_taylor(3, &mut budget).unwrap();
        let mut state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(1, 2, -1)]).unwrap(),
                &mut budget,
            )
            .unwrap(),
        );
        for step in 0..2 {
            let report = program.certify_noise_step(
                state,
                FlowNoiseRequest {
                    end: interval(step + 1, step + 1, -3),
                    candidate: BoxN::new([interval(0, 2, 0)]).unwrap(),
                    maximum_noise: 2,
                },
                &mut budget,
            );
            assert!(
                matches!(report.status(), FlowNoiseStatus::Certified),
                "{:?}",
                report.status()
            );
            if step == 1 {
                assert!(
                    report
                        .evidence()
                        .inputs()
                        .contains(&FlowNoiseInput::Residual(0))
                );
            }
            state = report.end_state().unwrap().clone();
            assert_ne!(state.residual().unwrap()[0], interval(0, 0, 0));
            if step == 0 {
                use hyperdynamics::field::{picard::PicardFailure, taylor_flow::FlowTaylorError};
                let coarse = report.evidence().flow().unwrap().end_enclosure().unwrap();
                assert!(
                    !state
                        .enclosure()
                        .subset_of(coarse, &mut DyadicBudget::new(16384, 1_000_000_000))
                        .unwrap()
                );
                let stationary = ExpressionProgram::compile(
                    &[Expr::int(0)],
                    &[x, time],
                    &mut ExpressionBudget::new(1000, 10000),
                )
                .unwrap();
                let stationary = ExpressionField::<1>::from_definition(
                    FieldDefinition {
                        source: stationary.definition().clone(),
                        time,
                        state: vec![x],
                    },
                    &mut budget,
                )
                .unwrap()
                .flow_taylor(1, &mut budget)
                .unwrap();
                let clipped = stationary.certify_noise_step(
                    state.clone(),
                    FlowNoiseRequest {
                        end: interval(2, 2, -3),
                        candidate: coarse.clone(),
                        maximum_noise: 2,
                    },
                    &mut budget,
                );
                assert!(matches!(
                    clipped.status(),
                    FlowNoiseStatus::Rejected(FlowNoiseError::Flow(FlowTaylorError::Picard(
                        PicardFailure::InitialOutsideCandidate
                    )))
                ));
                assert!(clipped.end_state().is_none());
                let complete_outer = stationary.certify_noise_step(
                    state.clone(),
                    FlowNoiseRequest {
                        end: interval(2, 2, -3),
                        candidate: state.enclosure().clone(),
                        maximum_noise: 2,
                    },
                    &mut budget,
                );
                assert!(
                    matches!(complete_outer.status(), FlowNoiseStatus::Certified),
                    "{:?}",
                    complete_outer.status()
                );
            }
            let actual = state.enclosure().lanes().unwrap()[0]
                .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
                .unwrap();
            let elapsed = Rational::from(step + 1) / Rational::from(8);
            // x(t)=x0/(1-t*x0), monotone for every x0 in [1/2,1].
            for initial in [Rational::from(1) / Rational::from(2), Rational::from(1)] {
                let exact = &initial / (Rational::from(1) - &elapsed * &initial);
                assert!(actual.contains_point(&exact), "{actual:?} misses {exact:?}");
            }
        }
    }

    #[test]
    fn invalid_time_precedes_empty_and_empty_steps_remain_valid_inputs() {
        let mut budget = budget();
        let mut zero_budget = EnclosureBudget::new(EnclosurePolicy {
            graph_work: 0,
            graph_entries: 0,
            scalar_work: 0,
            ..EnclosurePolicy::default()
        });
        let empty =
            FlowNoiseState::<2>::new(interval(0, 0, 0), BoxN::empty().unwrap(), &mut zero_budget)
                .unwrap();
        assert!(empty.enclosure().is_empty());
        assert!(matches!(
            FlowNoiseState::<2>::new(interval(0, 1, 0), BoxN::empty().unwrap(), &mut budget),
            Err(FlowNoiseError::NonPointTime)
        ));
        let program = shear(1, &mut budget).flow_taylor(1, &mut budget).unwrap();
        let mut state = Arc::new(
            FlowNoiseState::new(interval(0, 0, 0), BoxN::empty().unwrap(), &mut budget).unwrap(),
        );
        let invalid = program.certify_noise_step(
            state.clone(),
            FlowNoiseRequest {
                end: interval(0, 0, 0),
                candidate: BoxN::empty().unwrap(),
                maximum_noise: 2,
            },
            &mut budget,
        );
        assert!(matches!(invalid.status(), FlowNoiseStatus::Rejected(_)));
        assert!(invalid.end_state().is_none());
        for step in 0..2 {
            let report = program.certify_noise_step(
                state,
                FlowNoiseRequest {
                    end: interval(step + 1, step + 1, -2),
                    candidate: BoxN::empty().unwrap(),
                    maximum_noise: 0,
                },
                &mut budget,
            );
            assert!(matches!(report.status(), FlowNoiseStatus::Empty));
            state = report.end_state().unwrap().clone();
            assert!(state.enclosure().is_empty());
            assert!(state.center().is_none());
        }
    }

    #[test]
    fn zero_dimensional_unit_does_not_become_empty() {
        let mut budget = budget();
        let time = SymbolId(10);
        let source =
            ExpressionProgram::compile(&[], &[time], &mut ExpressionBudget::new(1000, 10000))
                .unwrap();
        let field = ExpressionField::<0>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![],
            },
            &mut budget,
        )
        .unwrap();
        let program = field.flow_taylor(1, &mut budget).unwrap();
        let state = Arc::new(
            FlowNoiseState::new(interval(0, 0, 0), BoxN::new([]).unwrap(), &mut budget).unwrap(),
        );
        let report = program.certify_noise_step(
            state,
            FlowNoiseRequest {
                end: interval(1, 1, -2),
                candidate: BoxN::new([]).unwrap(),
                maximum_noise: 0,
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseStatus::Certified),
            "{:?}",
            report.status()
        );
        let end = report.end_state().unwrap();
        assert!(!end.enclosure().is_empty());
        assert_eq!(end.center(), Some(&[]));
    }

    #[test]
    fn resource_failure_and_cancellation_publish_no_state_and_retry_succeeds() {
        let mut ordinary = budget();
        let program = shear(1, &mut ordinary)
            .flow_taylor(1, &mut ordinary)
            .unwrap();
        let state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap(),
                &mut ordinary,
            )
            .unwrap(),
        );
        let request = FlowNoiseRequest {
            end: interval(1, 1, -2),
            candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
            maximum_noise: 2,
        };
        let mut tiny = EnclosureBudget::new(EnclosurePolicy {
            graph_work: 0,
            ..EnclosurePolicy::default()
        });
        let report = program.certify_noise_step(state.clone(), request.clone(), &mut tiny);
        assert!(matches!(
            report.status(),
            FlowNoiseStatus::Rejected(FlowNoiseError::Field(FieldError::Expression(
                ExpressionError::WorkExhausted
            )))
        ));
        assert!(report.end_state().is_none());
        assert!(report.evidence().flow().is_none());
        let token = hypersolve::work_budget::CancellationToken::new();
        token.cancel();
        let mut cancelled = budget();
        cancelled.set_cancellation(Some(token));
        let report = program.certify_noise_step(state.clone(), request.clone(), &mut cancelled);
        assert!(matches!(
            report.status(),
            FlowNoiseStatus::Rejected(FlowNoiseError::Field(FieldError::Expression(
                ExpressionError::Cancelled
            )))
        ));
        assert!(report.end_state().is_none());
        let report = program.certify_noise_step(state.clone(), request, &mut budget());
        assert!(matches!(report.status(), FlowNoiseStatus::Certified));
        assert!(state.evidence().is_none());
        assert_eq!(
            state.enclosure().lanes(),
            Some(&[interval(-1, 1, 0), interval(-1, 1, 0)])
        );
    }

    #[test]
    fn final_materialization_exhaustion_keeps_proofs_but_exposes_no_endpoint() {
        let mut ordinary = budget();
        let program = shear(1, &mut ordinary)
            .flow_taylor(1, &mut ordinary)
            .unwrap();
        let state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap(),
                &mut ordinary,
            )
            .unwrap(),
        );
        let request = FlowNoiseRequest {
            end: interval(1, 1, -2),
            candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
            maximum_noise: 2,
        };
        let warm = program.certify_noise_step(state.clone(), request.clone(), &mut budget());
        assert!(matches!(warm.status(), FlowNoiseStatus::Certified));
        let mut measured = budget();
        let completed = program.certify_noise_step(state.clone(), request.clone(), &mut measured);
        assert!(matches!(completed.status(), FlowNoiseStatus::Certified));
        let graph_work = measured.policy().graph_work - measured.remaining_graph_work();
        let scalar_work = measured.policy().scalar_work - measured.remaining_scalar_work();
        for policy in [
            EnclosurePolicy {
                graph_work: graph_work - 1,
                ..EnclosurePolicy::default()
            },
            EnclosurePolicy {
                scalar_work: scalar_work - 1,
                ..EnclosurePolicy::default()
            },
        ] {
            let failed = program.certify_noise_step(
                state.clone(),
                request.clone(),
                &mut EnclosureBudget::new(policy),
            );
            assert!(
                matches!(
                    failed.status(),
                    FlowNoiseStatus::Rejected(
                        FlowNoiseError::Field(FieldError::Expression(
                            ExpressionError::WorkExhausted
                        )) | FlowNoiseError::Field(FieldError::Lattice(
                            hyperlattice_interval::IntervalLatticeError::Scalar(
                                hyperinterval::IntervalError::WorkExhausted
                            )
                        ))
                    )
                ),
                "{:?}",
                failed.status()
            );
            assert!(failed.evidence().noise().is_some());
            assert!(failed.end_state().is_none());
        }
        let retry = program.certify_noise_step(state, request, &mut budget());
        assert_eq!(
            retry.end_state().unwrap().enclosure(),
            completed.end_state().unwrap().enclosure()
        );
    }

    #[test]
    fn unique_history_drops_iteratively_with_a_retained_intermediate_state() {
        let mut budget = budget();
        let time = SymbolId(10);
        let source =
            ExpressionProgram::compile(&[], &[time], &mut ExpressionBudget::new(1000, 10000))
                .unwrap();
        let field = ExpressionField::<0>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![],
            },
            &mut budget,
        )
        .unwrap();
        let program = field.flow_taylor(1, &mut budget).unwrap();
        let mut state = Arc::new(
            FlowNoiseState::new(interval(0, 0, 0), BoxN::new([]).unwrap(), &mut budget).unwrap(),
        );
        let mut retained = None;
        for step in 0..512 {
            let report = program.certify_noise_step(
                state,
                FlowNoiseRequest {
                    end: interval(step + 1, step + 1, 0),
                    candidate: BoxN::new([]).unwrap(),
                    maximum_noise: 0,
                },
                &mut budget,
            );
            assert!(matches!(report.status(), FlowNoiseStatus::Certified));
            state = report.end_state().unwrap().clone();
            if step == 255 {
                retained = Some(state.clone());
            }
        }
        std::thread::Builder::new()
            .stack_size(64 * 1024)
            .spawn(move || drop(state))
            .unwrap()
            .join()
            .unwrap();
        let retained = retained.unwrap();
        assert_eq!(retained.time(), &interval(256, 256, 0));
        assert!(!retained.enclosure().is_empty());
        std::thread::Builder::new()
            .stack_size(64 * 1024)
            .spawn(move || drop(retained))
            .unwrap()
            .join()
            .unwrap();
    }

    #[test]
    fn oscillator_endpoints_enclose_independent_alternating_series() {
        use hyperinterval::DyadicBudget;
        use hyperreal::Rational as Q;
        let time = SymbolId(10);
        let x = SymbolId(20);
        let y = SymbolId(30);
        let source = ExpressionProgram::compile(
            &[Expr::anonymous_symbol(y), -Expr::anonymous_symbol(x)],
            &[y, x, time],
            &mut ExpressionBudget::new(100_000, 1_000_000),
        )
        .unwrap();
        let mut budget = budget();
        let field = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut budget,
        )
        .unwrap();
        let program = field.flow_taylor(3, &mut budget).unwrap();
        let mut state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(1, 1, 0), interval(0, 0, 0)]).unwrap(),
                &mut budget,
            )
            .unwrap(),
        );
        for step in 0..2 {
            let report = program.certify_noise_step(
                state,
                FlowNoiseRequest {
                    end: interval(step + 1, step + 1, -3),
                    candidate: BoxN::new([interval(0, 2, 0), interval(-1, 1, 0)]).unwrap(),
                    maximum_noise: 4,
                },
                &mut budget,
            );
            assert!(
                matches!(report.status(), FlowNoiseStatus::Certified),
                "{:?}",
                report.status()
            );
            state = report.end_state().unwrap().clone();
            let t = Q::from(step + 1) / Q::from(8);
            for row in 0..2 {
                let (sum, upper) = flow_oracles::alternating_trig(&t, row == 1);
                let bounds = state.enclosure().lanes().unwrap()[row]
                    .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000))
                    .unwrap();
                for exact_bound in if row == 0 {
                    [sum, upper]
                } else {
                    [-upper, -sum]
                } {
                    assert!(
                        bounds.contains_point(&exact_bound),
                        "{bounds:?} misses {exact_bound:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn original_time_pole_rejects_before_affine_endpoint_publication() {
        let time = SymbolId(10);
        let x = SymbolId(20);
        let source = ExpressionProgram::compile(
            &[Expr::int(1)
                / (Expr::anonymous_symbol(time)
                    - Expr::real(hyperreal::Real::from(
                        hyperreal::Rational::from(1) / hyperreal::Rational::from(8),
                    )))],
            &[time, x],
            &mut ExpressionBudget::new(100_000, 1_000_000),
        )
        .unwrap();
        let mut budget = budget();
        let field = ExpressionField::<1>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x],
            },
            &mut budget,
        )
        .unwrap();
        let state = Arc::new(
            FlowNoiseState::new(
                interval(0, 0, 0),
                BoxN::new([interval(0, 0, 0)]).unwrap(),
                &mut budget,
            )
            .unwrap(),
        );
        let report = field
            .flow_taylor(1, &mut budget)
            .unwrap()
            .certify_noise_step(
                state,
                FlowNoiseRequest {
                    end: interval(1, 1, -2),
                    candidate: BoxN::new([interval(-100, 100, 0)]).unwrap(),
                    maximum_noise: 1,
                },
                &mut budget,
            );
        assert!(matches!(
            report.status(),
            FlowNoiseStatus::Rejected(FlowNoiseError::Flow(_))
        ));
        assert!(matches!(
            report.evidence().flow().unwrap().status(),
            FlowTaylorStatus::Rejected(_)
        ));
        assert!(report.evidence().noise().is_none());
        assert!(report.end_state().is_none());
    }

    #[cfg(feature = "serde")]
    type Replay = (
        DyadicInterval,
        BoxN<DyadicInterval, 2>,
        Vec<(FieldDefinition, FlowNoiseRequest<2>)>,
    );

    #[cfg(feature = "serde")]
    fn replay(data: Replay) -> Vec<u8> {
        let (time, initial, steps) = data;
        let mut budget = budget();
        let mut state = Arc::new(FlowNoiseState::new(time, initial.clone(), &mut budget).unwrap());
        for (definition, request) in steps {
            let field = ExpressionField::<2>::from_definition(definition, &mut budget).unwrap();
            let report = field
                .flow_taylor(1, &mut budget)
                .unwrap()
                .certify_noise_step(state, request, &mut budget);
            assert!(matches!(report.status(), FlowNoiseStatus::Certified));
            state = report.end_state().unwrap().clone();
        }
        assert_eq!(state.enclosure(), &initial);
        serde_json::to_vec(&(state.time(), state.enclosure())).unwrap()
    }

    #[test]
    #[cfg(feature = "serde")]
    fn fresh_process_replays_original_inputs() {
        use hyperdynamics::field::EnclosedVectorField;
        const ENV: &str = "CSGRS_FLOW_NOISE_REPLAY_DIR";
        if let Some(path) = std::env::var_os(ENV) {
            let path = std::path::PathBuf::from(path);
            let data: Replay =
                serde_json::from_slice(&std::fs::read(path.join("input.json")).unwrap()).unwrap();
            std::fs::write(path.join("output.json"), replay(data)).unwrap();
            return;
        }
        let mut budget = budget();
        let mut steps = Vec::new();
        for (step, sign) in [(0, 1), (1, -1)] {
            let identity = shear(sign, &mut budget).identity();
            steps.push((
                FieldDefinition {
                    source: identity.source().clone(),
                    time: identity.time(),
                    state: identity.state().to_vec(),
                },
                FlowNoiseRequest {
                    end: interval(step + 1, step + 1, -2),
                    candidate: BoxN::new([interval(-2, 2, 0), interval(-1, 1, 0)]).unwrap(),
                    maximum_noise: 2,
                },
            ));
        }
        let data = (
            interval(0, 0, 0),
            BoxN::new([interval(-1, 1, 0), interval(-1, 1, 0)]).unwrap(),
            steps,
        );
        let expected = replay(data.clone());
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path =
            std::env::temp_dir().join(format!("flow-noise-replay-{}-{nonce}", std::process::id()));
        std::fs::create_dir(&path).unwrap();
        std::fs::write(path.join("input.json"), serde_json::to_vec(&data).unwrap()).unwrap();
        let output = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "closure::fresh_process_replays_original_inputs",
                "--nocapture",
            ])
            .env(ENV, &path)
            .output()
            .unwrap();
        assert!(output.status.success(), "{output:?}");
        assert_eq!(std::fs::read(path.join("output.json")).unwrap(), expected);
        std::fs::remove_dir_all(path).unwrap();
    }
}

#[cfg(all(feature = "flow-noise", feature = "taylor-evolution"))]
mod two_rate {
    use super::*;
    use hyperdynamics::field::{
        evolution::{EvolutionOptions, EvolutionRequest},
        taylor_flow::{
            evolution::FlowTaylorEvolutionStatus,
            noise::{FlowNoiseInput, FlowNoiseRequest, FlowNoiseState, FlowNoiseStatus},
        },
    };
    use hyperinterval::DyadicBudget;
    use hyperreal::Rational as Q;
    use std::sync::Arc;

    fn arithmetic() -> DyadicBudget {
        DyadicBudget::new(16384, 1_000_000_000)
    }

    fn bounds(time: i64, rate: i64, radius: i64) -> (Q, Q) {
        let (lo, hi) = crate::exponential_oracle::exp_bounds(Q::from(time * rate) / Q::from(64));
        // Reciprocation reverses the positive exp bounds. Scaling by the
        // positive initial interval gives exact independent solution bounds.
        (
            Q::from(1024 - radius) / Q::from(1024) / hi,
            Q::from(1024 + radius) / Q::from(1024) / lo,
        )
    }

    fn contains(value: &DyadicInterval, lo: &Q, hi: &Q) {
        let range = value.to_rational(&mut arithmetic()).unwrap();
        assert!(
            range.contains_point(lo),
            "missing lower extremum {lo:?} from {range:?}"
        );
        assert!(
            range.contains_point(hi),
            "missing upper extremum {hi:?} from {range:?}"
        );
    }

    #[test]
    fn full_two_rate_noise_chains_enclose_continuous_point_and_box_flows() {
        let (time, x, y) = (SymbolId(0), SymbolId(1), SymbolId(2));
        let source = ExpressionProgram::compile(
            &[
                -Expr::anonymous_symbol(x),
                Expr::int(-8) * Expr::anonymous_symbol(y),
            ],
            &[time, x, y],
            &mut ExpressionBudget::new(1_000_000, 100_000_000),
        )
        .unwrap();
        let mut preparation = EnclosureBudget::new(EnclosurePolicy::default());
        let program = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut preparation,
        )
        .unwrap()
        .flow_taylor(3, &mut preparation)
        .unwrap();
        for radius in [0, 1] {
            let initial = BoxN::new([
                interval(1024 - radius, 1024 + radius, -10),
                interval(1024 - radius, 1024 + radius, -10),
            ])
            .unwrap();
            let mut proposal_budget = EnclosureBudget::new(EnclosurePolicy::default());
            let proposals = program.integrate(
                EvolutionRequest {
                    start: interval(0, 0, 0),
                    end: interval(1, 1, -2),
                    initial: initial.clone(),
                },
                EvolutionOptions {
                    maximum_step: Some(interval(1, 1, -6)),
                    ..Default::default()
                },
                &mut proposal_budget,
            );
            assert!(matches!(
                proposals.status(),
                FlowTaylorEvolutionStatus::Completed
            ));
            assert_eq!(proposals.segments().len(), 16);
            let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
            let mut state = Arc::new(
                FlowNoiseState::new(interval(0, 0, 0), initial.clone(), &mut budget).unwrap(),
            );
            let mut tube = initial.lanes().unwrap().clone();
            for (index, proposal) in proposals.segments().enumerate() {
                let previous = state.clone();
                let request = proposal.picard().proposal();
                assert_eq!(request.start, interval(index as i64, index as i64, -6));
                assert_eq!(
                    request.end,
                    interval(index as i64 + 1, index as i64 + 1, -6)
                );
                let report = program.certify_noise_step(
                    state,
                    FlowNoiseRequest {
                        end: request.end.clone(),
                        candidate: request.candidate.clone(),
                        maximum_noise: 2,
                    },
                    &mut budget,
                );
                assert!(
                    matches!(report.status(), FlowNoiseStatus::Certified),
                    "{:?}",
                    report.status()
                );
                let evidence = report.evidence();
                assert!(Arc::ptr_eq(evidence.initial(), &previous));
                let flow = evidence.flow().unwrap();
                assert!(matches!(flow.status(), FlowTaylorStatus::Certified));
                assert_eq!(&flow.picard().proposal().initial, previous.enclosure());
                assert_eq!(&flow.picard().proposal().start, previous.time());
                if index > 0 {
                    assert!(evidence.inputs().contains(&FlowNoiseInput::Residual(0)));
                    assert!(evidence.inputs().contains(&FlowNoiseInput::Residual(1)));
                }
                state = report.end_state().unwrap().clone();
                assert!(Arc::ptr_eq(state.evidence().unwrap(), evidence));
                assert_eq!(state.time(), &request.end);
                assert!(state.columns().unwrap().len() <= 2);
                for (lane, rate) in [1, 8].into_iter().enumerate() {
                    let (end_lo, end_hi) = bounds(index as i64 + 1, rate, radius);
                    let (_, start_hi) = bounds(index as i64, rate, radius);
                    contains(&state.enclosure().lanes().unwrap()[lane], &end_lo, &end_hi);
                    let piece = &flow.tube().unwrap().lanes().unwrap()[lane];
                    // Positive decays are monotone throughout each whole step.
                    contains(piece, &end_lo, &start_hi);
                    tube[lane] = tube[lane].hull(piece, &mut arithmetic()).unwrap();
                    assert_ne!(state.residual().unwrap()[lane], interval(0, 0, 0));
                }
            }
            assert_eq!(state.time(), &interval(1, 1, -2));
            for (lane, rate) in [1, 8].into_iter().enumerate() {
                let (end_lo, _) = bounds(16, rate, radius);
                let (_, start_hi) = bounds(0, rate, radius);
                contains(&tube[lane], &end_lo, &start_hi);
                assert!(
                    tube[lane]
                        .width(&mut arithmetic())
                        .unwrap()
                        .subset_of(&interval(0, 1, 0), &mut arithmetic())
                        .unwrap()
                );
                let width = state.enclosure().lanes().unwrap()[lane]
                    .width(&mut arithmetic())
                    .unwrap();
                let exponent = if radius == 0 {
                    [-29, -17][lane]
                } else {
                    [-9, -11][lane]
                };
                assert!(
                    width
                        .subset_of(&interval(0, 1, exponent), &mut arithmetic())
                        .unwrap()
                );
            }
            println!(
                "two_rate radius={radius}; end={:?}; tube={tube:?}; graph_work={}; scalar_work={}",
                state.enclosure(),
                budget.policy().graph_work - budget.remaining_graph_work(),
                budget.policy().scalar_work - budget.remaining_scalar_work()
            );
        }
    }

    #[test]
    #[cfg(feature = "flow-noise-evolution")]
    fn automatic_two_rate_noise_horizon_preserves_full_tubes_and_residuals() {
        use hyperdynamics::field::taylor_flow::noise::evolution::{
            FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus,
        };
        let (time, x, y) = (SymbolId(0), SymbolId(1), SymbolId(2));
        let source = ExpressionProgram::compile(
            &[
                -Expr::anonymous_symbol(x),
                Expr::int(-8) * Expr::anonymous_symbol(y),
            ],
            &[time, x, y],
            &mut ExpressionBudget::new(1_000_000, 100_000_000),
        )
        .unwrap();
        let mut preparation = EnclosureBudget::new(EnclosurePolicy::default());
        let program = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut preparation,
        )
        .unwrap()
        .flow_taylor(3, &mut preparation)
        .unwrap();
        for radius in [0, 1] {
            let initial = BoxN::new([
                interval(1024 - radius, 1024 + radius, -10),
                interval(1024 - radius, 1024 + radius, -10),
            ])
            .unwrap();
            let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
            let mut previous = Arc::new(
                FlowNoiseState::new(interval(0, 0, 0), initial.clone(), &mut budget).unwrap(),
            );
            let report = program.integrate_noise(
                previous.clone(),
                interval(1, 1, -2),
                FlowNoiseEvolutionOptions {
                    maximum_noise: 2,
                    native: EvolutionOptions {
                        maximum_step: Some(interval(1, 1, -6)),
                        ..Default::default()
                    },
                },
                &mut budget,
            );
            assert!(
                matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
                "{:?}",
                report.status()
            );
            assert_eq!(report.segments().len(), 16);
            let mut tube = initial.lanes().unwrap().clone();
            for (index, segment) in report.segments().enumerate() {
                let evidence = segment.evidence();
                assert!(Arc::ptr_eq(evidence.initial(), &previous));
                let flow = evidence.flow().unwrap();
                assert!(matches!(flow.status(), FlowTaylorStatus::Certified));
                assert_eq!(&flow.picard().proposal().initial, previous.enclosure());
                assert_eq!(&flow.picard().proposal().start, previous.time());
                let next = segment.end_state().unwrap();
                assert!(Arc::ptr_eq(next.evidence().unwrap(), evidence));
                assert_eq!(
                    next.time(),
                    &interval(index as i64 + 1, index as i64 + 1, -6)
                );
                assert!(next.columns().unwrap().len() <= 2);
                for (lane, rate) in [1, 8].into_iter().enumerate() {
                    if index > 0 {
                        assert!(evidence.inputs().contains(&FlowNoiseInput::Residual(lane)));
                    }
                    let (end_lo, end_hi) = bounds(index as i64 + 1, rate, radius);
                    let (_, start_hi) = bounds(index as i64, rate, radius);
                    contains(&next.enclosure().lanes().unwrap()[lane], &end_lo, &end_hi);
                    let piece = &flow.tube().unwrap().lanes().unwrap()[lane];
                    contains(piece, &end_lo, &start_hi);
                    tube[lane] = tube[lane].hull(piece, &mut arithmetic()).unwrap();
                    assert_ne!(next.residual().unwrap()[lane], interval(0, 0, 0));
                }
                previous = next.clone();
            }
            assert!(Arc::ptr_eq(report.end_state().unwrap(), &previous));
            for (lane, rate) in [1, 8].into_iter().enumerate() {
                let (end_lo, _) = bounds(16, rate, radius);
                let (_, start_hi) = bounds(0, rate, radius);
                contains(&tube[lane], &end_lo, &start_hi);
                assert!(
                    tube[lane]
                        .width(&mut arithmetic())
                        .unwrap()
                        .subset_of(&interval(0, 1, 0), &mut arithmetic())
                        .unwrap()
                );
                let exponent = if radius == 0 {
                    [-29, -17][lane]
                } else {
                    [-9, -11][lane]
                };
                assert!(
                    previous.enclosure().lanes().unwrap()[lane]
                        .width(&mut arithmetic())
                        .unwrap()
                        .subset_of(&interval(0, 1, exponent), &mut arithmetic())
                        .unwrap()
                );
            }
        }
    }
}

#[cfg(all(feature = "flow-noise", feature = "taylor-evolution"))]
#[path = "support/exponential_oracle.rs"]
mod exponential_oracle;

#[cfg(all(feature = "serde", feature = "flow-noise"))]
#[path = "support/replay.rs"]
mod replay;
