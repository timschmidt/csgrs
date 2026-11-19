#![cfg(feature = "rk-proposal")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    FieldError,
    evolution::{EvolutionOptions, EvolutionRequest, EvolutionStatus},
    picard::{PicardFailure, PicardStatus},
    proposal::*,
};
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::BoxN;
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionError, ExpressionProgram, FloatProposalError,
    },
    work_budget::CancellationToken,
};
const T: SymbolId = SymbolId(100);
const X: SymbolId = SymbolId(0);
const V: SymbolId = SymbolId(1);
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn point(a: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::point(a.into(), exponent).unwrap()
}
fn field<const N: usize>(outputs: &[Expr]) -> ExpressionField<N> {
    let state: Vec<_> = (0..N).map(|i| SymbolId(i as u32)).collect();
    let mut variables = state.clone();
    variables.reverse();
    variables.insert(N / 2, T);
    let source = ExpressionProgram::compile(
        outputs,
        &variables,
        &mut ExpressionBudget::new(100000, 10000000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state,
        },
        &mut b(),
    )
    .unwrap()
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
fn certified<const N: usize>(a: &Rk4Attempt<N>) {
    assert!(
        matches!(a.proof().status(), PicardStatus::Certified),
        "{:?}",
        a.proof().status()
    );
    assert_eq!(a.proof().identity(), &a.proposer().field().identity());
    assert!(a.proof().end_enclosure().is_some());
    assert!(a.proof().tube().is_some());
    assert_eq!(a.trace().stages().len(), 4);
    assert!(a.trace().stages().iter().all(|s| s.derivative().is_some()));
}
fn contains(value: &DyadicInterval, expected: &Rational) {
    assert!(
        value
            .to_rational(&mut arithmetic())
            .unwrap()
            .contains_point(expected),
        "{expected:?} outside {value:?}"
    );
}

#[test]
fn subnormal_proposal_replay_preserves_the_initial_set_across_steps() {
    let stationary = field::<1>(&[Expr::int(0)]).rk4_proposer(&mut b()).unwrap();
    let translating = field::<1>(&[Expr::int(1)]).rk4_proposer(&mut b()).unwrap();
    for (lo, hi) in [(-3_i64, -3_i64), (1, 3), (-3, 1), (1, 2)] {
        let grid = |a: i64, b: i64| DyadicInterval::new(a.into(), b.into(), -1074).unwrap();
        let initial = grid(lo, hi);
        let first = stationary
            .propose(
                request(point(1, -1074), [initial.clone()]),
                Default::default(),
                &mut b(),
            )
            .unwrap()
            .certify(&mut b());
        certified(&first);
        let endpoint = first.proof().end_enclosure().unwrap();
        assert_eq!(endpoint.lanes(), Some(&[initial]));

        let next = EvolutionRequest {
            start: point(1, -1074),
            end: point(5, -1074),
            initial: endpoint.clone(),
        };
        let second = translating
            .propose(next, Default::default(), &mut b())
            .unwrap()
            .certify(&mut b());
        certified(&second);
        assert_eq!(
            second.proof().end_enclosure().unwrap().lanes(),
            Some(&[grid(lo + 4, hi + 4)])
        );
    }
}

#[test]
fn zero_flow_preserves_subnormal_proposal_width_before_exact_replay() {
    let proposer = field::<1>(&[Expr::int(0)]).rk4_proposer(&mut b()).unwrap();
    let initial = DyadicInterval::new(1.into(), 3.into(), -1074).unwrap();
    let req = request(point(1, -2), [initial.clone()]);
    let candidate = proposer
        .propose(
            req,
            Rk4Options {
                relative_padding: 0.0,
                absolute_padding: 0.0,
            },
            &mut b(),
        )
        .unwrap();
    let (lower, upper) = candidate.trace().bounds()[0].estimates().unwrap();
    assert_eq!((lower.to_bits(), upper.to_bits()), (1, 3));
    let attempt = candidate.certify(&mut b());
    certified(&attempt);
    assert_eq!(
        attempt.proof().end_enclosure().unwrap().lanes(),
        Some(&[initial])
    );
}

#[test]
fn constant_flow_preserves_subnormal_stage_coordinates_and_time() {
    let proposer = field::<1>(&[Expr::int(0)]).rk4_proposer(&mut b()).unwrap();
    let initial = point(-3, -1074);
    let req = EvolutionRequest {
        start: point(1, -1074),
        end: point(5, -1074),
        initial: BoxN::new([initial.clone()]).unwrap(),
    };
    let attempt = proposer
        .propose(req, Default::default(), &mut b())
        .unwrap()
        .certify(&mut b());
    certified(&attempt);
    for (stage, time_bits) in attempt.trace().stages().iter().zip([1, 3, 3, 5]) {
        assert_eq!(stage.time().to_bits(), time_bits);
        assert_eq!(stage.state()[0].to_bits(), (-f64::from_bits(3)).to_bits());
    }
    assert_eq!(
        attempt.proof().end_enclosure().unwrap().lanes(),
        Some(&[initial])
    );
}

#[test]
fn uncertain_constant_flows_keep_exact_inputs_and_feed_subsequent_operations() {
    for c in -3..=3 {
        for initial in -3..=3 {
            let source = field::<1>(&[Expr::int(c)]);
            let prepared = source.rk4_proposer(&mut b()).unwrap();
            let req = request(point(1, -2), [d(initial, initial + 1)]);
            let candidate = prepared
                .propose(req.clone(), Default::default(), &mut b())
                .unwrap();
            assert_eq!(candidate.proposal().start, req.start);
            assert_eq!(candidate.proposal().end, req.end);
            assert_eq!(candidate.proposal().initial, req.initial);
            assert!(
                req.initial
                    .subset_of(&candidate.proposal().candidate, &mut arithmetic())
                    .unwrap()
            );
            let a = candidate.certify(&mut b());
            certified(&a);
            assert_eq!(
                a.proof().end_enclosure().unwrap().lanes(),
                Some(&[DyadicInterval::new(
                    (4 * initial + c).into(),
                    (4 * (initial + 1) + c).into(),
                    -2
                )
                .unwrap()])
            );
            let next = source.integrate(
                EvolutionRequest {
                    start: req.end,
                    end: point(1, -1),
                    initial: a.proof().end_enclosure().unwrap().clone(),
                },
                EvolutionOptions::default(),
                &mut b(),
            );
            assert!(matches!(next.status(), EvolutionStatus::Completed));
            assert_eq!(
                next.end_enclosure().unwrap().lanes(),
                Some(&[DyadicInterval::new(
                    (2 * initial + c).into(),
                    (2 * (initial + 1) + c).into(),
                    -1
                )
                .unwrap()])
            );
        }
    }
}

#[test]
fn rk_polynomial_stages_and_verified_nonlinear_flows_have_independent_controls() {
    let source = field::<1>(&[Expr::anonymous_symbol(X)]);
    let proposal = source
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -2), [d(1, 1)]),
            Default::default(),
            &mut b(),
        )
        .unwrap();
    let stages = proposal.trace().stages();
    for (stage, (t, x)) in stages.iter().zip([
        (0.0, 1.0),
        (0.125, 1.125),
        (0.125, 1.140625),
        (0.25, 1.28515625),
    ]) {
        assert_eq!(stage.time(), t);
        assert_eq!(stage.state(), &[x]);
        assert_eq!(stage.derivative(), Some(&[x]));
    }
    // The fourth-degree RK stability polynomial at h=1/4 is exactly 7889/6144.
    assert_eq!(proposal.trace().end_estimate(), Some(&[7889.0 / 6144.0]));
    let a = proposal.certify(&mut b());
    certified(&a);
    // Independent positive Taylor series and a geometric tail for exp(1/4).
    let t = Rational::fraction(1, 4).unwrap();
    let mut term = Rational::one();
    let mut sum = term.clone();
    for k in 1..=8 {
        term = term * &t / Rational::from(k);
        sum = &sum + &term;
    }
    let tail = term * &t / Rational::from(9) / (Rational::one() - &t / Rational::from(10));
    let end = &a.proof().end_enclosure().unwrap().lanes().unwrap()[0];
    contains(end, &sum);
    contains(end, &(sum + tail));

    let affine = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T) + Expr::int(3)]);
    let a = affine
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -2), [d(0, 0)]),
            Default::default(),
            &mut b(),
        )
        .unwrap()
        .certify(&mut b());
    certified(&a);
    contains(
        &a.proof().end_enclosure().unwrap().lanes().unwrap()[0],
        &Rational::fraction(13, 16).unwrap(),
    );

    let oscillator = field::<2>(&[Expr::anonymous_symbol(V), -Expr::anonymous_symbol(X)]);
    let a = oscillator
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -2), [d(1, 1), d(0, 0)]),
            Rk4Options {
                relative_padding: 0.0,
                absolute_padding: 0.125,
            },
            &mut b(),
        )
        .unwrap()
        .certify(&mut b());
    certified(&a);
    let end = a.proof().end_enclosure().unwrap().lanes().unwrap();
    // Alternating Taylor bounds for cos(1/4) and -sin(1/4).
    for n in [
        Rational::fraction(31, 32).unwrap(),
        Rational::fraction(5953, 6144).unwrap(),
    ] {
        contains(&end[0], &n);
    }
    for n in [
        Rational::fraction(-1, 4).unwrap(),
        Rational::fraction(-95, 384).unwrap(),
    ] {
        contains(&end[1], &n);
    }

    let riccati = field::<1>(&[Expr::anonymous_symbol(X).powi(2)]);
    let a = riccati
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -3), [d(1, 1)]),
            Default::default(),
            &mut b(),
        )
        .unwrap()
        .certify(&mut b());
    certified(&a);
    contains(
        &a.proof().end_enclosure().unwrap().lanes().unwrap()[0],
        &Rational::fraction(8, 7).unwrap(),
    );
}

#[test]
fn numerical_success_is_never_a_source_domain_or_trajectory_certificate() {
    let tiny = (Real::from(1) / Real::from(2).powi(100.into()).unwrap()).unwrap();
    let original = field::<1>(&[(Expr::int(1) - (Expr::int(1) + Expr::Constant(tiny))).sqrt()]);
    let proposal = original
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -2), [d(0, 0)]),
            Default::default(),
            &mut b(),
        )
        .unwrap();
    assert_eq!(proposal.trace().end_estimate(), Some(&[0.0]));
    let checked = proposal.certify(&mut b());
    assert!(matches!(
        checked.proof().status(),
        PicardStatus::Rejected(PicardFailure::Field(_))
    ));
    assert!(checked.proof().end_enclosure().is_none());
    assert!(checked.proof().tube().is_none());
    assert_eq!(checked.trace().stages().len(), 4);

    let original = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(X)]);
    let req = request(d(1, 1), [d(1, 1)]);
    let checked = original
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(req.clone(), Default::default(), &mut b())
        .unwrap()
        .certify(&mut b());
    assert!(matches!(
        checked.proof().status(),
        PicardStatus::Rejected(_)
    ));
    assert!(checked.proof().end_enclosure().is_none());
    assert!(matches!(
        original
            .integrate(req, Default::default(), &mut b())
            .status(),
        EvolutionStatus::Completed
    ));
}

#[test]
fn proposal_failure_preserves_original_request_stages_and_native_fallback() {
    let original = field::<1>(&[Expr::int(0)]);
    let prepared = original.rk4_proposer(&mut b()).unwrap();
    let close = d(1, 1).add(&point(1, -100), &mut arithmetic()).unwrap();
    for (req, expected) in [
        (
            EvolutionRequest {
                start: d(1, 1),
                end: close,
                initial: BoxN::new([d(0, 0)]).unwrap(),
            },
            false,
        ),
        (request(point(1, 2048), [d(0, 0)]), true),
    ] {
        let failure = prepared
            .propose(req.clone(), Default::default(), &mut b())
            .unwrap_err();
        if expected {
            assert!(matches!(
                failure.error(),
                Rk4Error::ProjectionUnavailable(ProposalInput::EndTime)
            ));
        } else {
            assert!(matches!(
                failure.error(),
                Rk4Error::PrimitiveTimeUnavailable
            ));
        }
        assert_eq!(failure.request().end, req.end);
        assert!(failure.trace().stages().is_empty());
        assert!(matches!(
            original
                .integrate(req, Default::default(), &mut b())
                .status(),
            EvolutionStatus::Completed
        ));
    }
    let original = field::<1>(&[Expr::int(1) / Expr::anonymous_symbol(X)]);
    let failure = original
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(point(1, -2), [d(0, 0)]),
            Default::default(),
            &mut b(),
        )
        .unwrap_err();
    assert!(matches!(
        failure.error(),
        Rk4Error::Estimate {
            stage: 0,
            source: FloatProposalError::NativeDomain { .. }
        }
    ));
    assert_eq!(failure.trace().stages().len(), 1);
    assert!(failure.trace().stages()[0].derivative().is_none());
    assert_eq!(failure.proposer().field().identity(), original.identity());
    let original = field::<1>(&[Expr::anonymous_symbol(X).powi(2)]);
    let failure = original
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(
            request(
                point(1, -2),
                [DyadicInterval::from_f64(f64::MAX, &mut arithmetic()).unwrap()],
            ),
            Default::default(),
            &mut b(),
        )
        .unwrap_err();
    assert!(matches!(
        failure.error(),
        Rk4Error::Estimate {
            stage: 0,
            source: FloatProposalError::NonFiniteResult { .. }
        }
    ));
    assert_eq!(failure.trace().stages().len(), 1);
}

#[test]
fn empty_unit_invalid_inputs_and_shared_budget_exhaustion_remain_distinct() {
    let original = field::<1>(&[Expr::int(1)]);
    let mut measured = b();
    let prepared = original.rk4_proposer(&mut measured).unwrap();
    let p = measured.policy();
    let preparation = p.graph_work - measured.remaining_graph_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: preparation,
        ..p
    });
    original.rk4_proposer(&mut exact).unwrap();
    assert_eq!(exact.remaining_graph_work(), 0);
    assert!(matches!(
        original.rk4_proposer(&mut EnclosureBudget::new(EnclosurePolicy {
            graph_work: preparation - 1,
            ..p
        })),
        Err(FloatProposalError::Expression(
            ExpressionError::WorkExhausted
        ))
    ));
    let req = request(point(1, -2), [d(0, 1)]);
    let mut measured = b();
    let a = prepared
        .propose(req.clone(), Default::default(), &mut measured)
        .unwrap()
        .certify(&mut measured);
    certified(&a);
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw,
        scalar_work: sw,
        ..p
    });
    certified(
        &prepared
            .propose(req.clone(), Default::default(), &mut exact)
            .unwrap()
            .certify(&mut exact),
    );
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        scalar_work: sw - 1,
        ..p
    });
    let a = prepared
        .propose(req.clone(), Default::default(), &mut short)
        .unwrap()
        .certify(&mut short);
    assert!(
        matches!(
            a.proof().status(),
            PicardStatus::Rejected(PicardFailure::Field(FieldError::Lattice(
                hyperlattice_interval::IntervalLatticeError::Scalar(IntervalError::WorkExhausted)
            )))
        ),
        "{:?}",
        a.proof().status()
    );
    assert!(a.proof().end_enclosure().is_none());
    assert_eq!(a.trace().stages().len(), 4);
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = b();
    cancelled.set_cancellation(Some(token));
    let failure = prepared
        .propose(req.clone(), Default::default(), &mut cancelled)
        .unwrap_err();
    assert!(matches!(
        failure.error(),
        Rk4Error::Field(FieldError::Expression(ExpressionError::Cancelled))
    ));
    assert!(failure.trace().stages().is_empty());
    let candidate = prepared
        .propose(req.clone(), Default::default(), &mut b())
        .unwrap();
    let a = candidate.certify(&mut cancelled);
    assert!(matches!(
        a.proof().status(),
        PicardStatus::Rejected(PicardFailure::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert_eq!(a.trace().stages().len(), 4);
    certified(
        &prepared
            .propose(req, Default::default(), &mut b())
            .unwrap()
            .certify(&mut b()),
    );
    let empty = EvolutionRequest {
        start: d(0, 0),
        end: d(1, 1),
        initial: BoxN::empty().unwrap(),
    };
    let a = prepared
        .propose(empty.clone(), Default::default(), &mut b())
        .unwrap()
        .certify(&mut b());
    assert!(matches!(a.proof().status(), PicardStatus::Empty));
    assert!(a.trace().stages().is_empty());
    for padding in [-1.0, -f64::from_bits(1), f64::NAN, f64::INFINITY] {
        for relative in [false, true] {
            assert!(matches!(
                prepared
                    .propose(
                        empty.clone(),
                        if relative {
                            Rk4Options {
                                relative_padding: padding,
                                ..Default::default()
                            }
                        } else {
                            Rk4Options {
                                absolute_padding: padding,
                                ..Default::default()
                            }
                        },
                        &mut b()
                    )
                    .unwrap_err()
                    .error(),
                Rk4Error::InvalidPadding
            ));
        }
    }
    let negative_zero = prepared
        .propose(
            empty.clone(),
            Rk4Options {
                relative_padding: -0.0,
                absolute_padding: -0.0,
            },
            &mut b(),
        )
        .unwrap();
    assert_eq!(
        negative_zero.trace().options().relative_padding.to_bits(),
        (-0.0f64).to_bits()
    );
    assert_eq!(
        negative_zero.trace().options().absolute_padding.to_bits(),
        (-0.0f64).to_bits()
    );
    assert!(matches!(
        negative_zero.certify(&mut b()).proof().status(),
        PicardStatus::Empty
    ));
    let mut invalid = empty;
    invalid.end = d(0, 0);
    assert!(matches!(
        prepared
            .propose(invalid, Default::default(), &mut b())
            .unwrap_err()
            .error(),
        Rk4Error::NonPositiveDuration
    ));
    let unit = field::<0>(&[])
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(request(d(1, 1), []), Default::default(), &mut b())
        .unwrap()
        .certify(&mut b());
    certified(&unit);
    assert!(!unit.proof().end_enclosure().unwrap().is_empty());
}

#[test]
fn proposal_budget_cutoffs_preserve_every_partial_stage_and_retry() {
    let source = field::<1>(&[Expr::anonymous_symbol(X)]);
    let prepared = source.rk4_proposer(&mut b()).unwrap();
    let req = request(point(1, -2), [d(1, 1)]);
    let mut measured = b();
    let expected = prepared
        .propose(req.clone(), Default::default(), &mut measured)
        .unwrap();
    let p = measured.policy();
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let mut prefixes = std::collections::BTreeSet::new();
    for work in 0..gw {
        let mut limited = EnclosureBudget::new(EnclosurePolicy {
            graph_work: work,
            ..p
        });
        let failure = prepared
            .propose(req.clone(), Default::default(), &mut limited)
            .unwrap_err();
        assert!(
            matches!(
                failure.error(),
                Rk4Error::Field(FieldError::Expression(ExpressionError::WorkExhausted))
                    | Rk4Error::Estimate {
                        source: FloatProposalError::Expression(ExpressionError::WorkExhausted),
                        ..
                    }
            ),
            "{:?}",
            failure.error()
        );
        assert_eq!(failure.request().start, req.start);
        assert_eq!(failure.request().end, req.end);
        assert_eq!(failure.request().initial, req.initial);
        assert_eq!(failure.proposer().field().identity(), source.identity());
        prefixes.insert(failure.trace().stages().len());
        let completed = failure
            .trace()
            .stages()
            .iter()
            .take_while(|s| s.derivative().is_some())
            .count();
        assert!(completed + 1 >= failure.trace().stages().len());
    }
    assert_eq!(prefixes, (0..=4).collect());
    let small = prepared
        .propose(
            req.clone(),
            Default::default(),
            &mut EnclosureBudget::new(EnclosurePolicy {
                graph_entries: 23,
                ..p
            }),
        )
        .unwrap_err();
    assert!(matches!(
        small.error(),
        Rk4Error::Field(FieldError::Expression(ExpressionError::SizeExhausted))
    ));
    assert!(small.trace().stages().is_empty());
    let small = prepared
        .propose(
            req.clone(),
            Default::default(),
            &mut EnclosureBudget::new(EnclosurePolicy {
                scalar_bits: 0,
                ..p
            }),
        )
        .unwrap_err();
    assert!(matches!(
        small.error(),
        Rk4Error::Field(FieldError::Scalar(IntervalError::SizeExhausted))
    ));
    assert!(small.trace().stages().is_empty());
    for work in [0, sw / 2, sw - 1] {
        let failure = prepared
            .propose(
                req.clone(),
                Default::default(),
                &mut EnclosureBudget::new(EnclosurePolicy {
                    scalar_work: work,
                    ..p
                }),
            )
            .unwrap_err();
        assert!(
            matches!(
                failure.error(),
                Rk4Error::Field(FieldError::Scalar(IntervalError::WorkExhausted))
                    | Rk4Error::Field(FieldError::Lattice(
                        hyperlattice_interval::IntervalLatticeError::Scalar(
                            IntervalError::WorkExhausted
                        )
                    ))
            ),
            "{:?}",
            failure.error()
        );
        assert_eq!(failure.request().initial, req.initial);
    }
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw * 2,
        scalar_work: sw * 2,
        ..p
    });
    for _ in 0..2 {
        let retry = prepared
            .propose(req.clone(), Default::default(), &mut exact)
            .unwrap();
        assert_eq!(retry.proposal().candidate, expected.proposal().candidate);
        assert_eq!(
            retry.trace().end_estimate(),
            expected.trace().end_estimate()
        );
    }
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    certified(&expected.certify(&mut b()));
}

#[test]
fn later_stage_failure_retains_the_failed_input_and_original_source() {
    // The initial derivative exists; the second RK sample hits the original pole.
    let req = request(d(2, 2), [d(-1, -1)]);
    // Negative derivative would move away from zero. Negate it to hit zero at h/2.
    let source = field::<1>(&[-(Expr::int(1) / Expr::anonymous_symbol(X))]);
    let failure = source
        .rk4_proposer(&mut b())
        .unwrap()
        .propose(req, Default::default(), &mut b())
        .unwrap_err();
    assert!(matches!(
        failure.error(),
        Rk4Error::Estimate {
            stage: 1,
            source: FloatProposalError::NativeDomain { .. }
        }
    ));
    assert_eq!(failure.trace().stages().len(), 2);
    assert_eq!(failure.trace().stages()[0].derivative(), Some(&[1.0]));
    assert_eq!(failure.trace().stages()[1].state(), &[0.0]);
    assert_eq!(failure.trace().stages()[1].derivative(), None);
    assert!(failure.trace().end_estimate().is_none());
    assert!(failure.trace().bounds().is_empty());
    assert_eq!(failure.proposer().field().identity(), source.identity());
    let huge = field::<1>(&[Expr::Constant(Real::from(2).powi(2048.into()).unwrap())]);
    assert!(matches!(
        huge.rk4_proposer(&mut b()),
        Err(FloatProposalError::ConstantUnavailable { .. })
    ));
    assert_eq!(huge.identity().state(), &[X]);
}

#[test]
fn rational_clock_rk_endpoint_composes_with_original_physical_time() {
    use hyperdynamics::field::clock::ClockStatus;
    let q = |n, d| Rational::fraction(n, d).unwrap();
    let source = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T) + Expr::int(3)]);
    let clocked = source.on_time_span(q(1, 3), q(2, 3), &mut b()).unwrap();
    let normalized = clocked.normalized_field();
    let prepared = normalized.rk4_proposer(&mut b()).unwrap();
    let mut initial = BoxN::new([d(0, 0)]).unwrap();
    for i in 0..4 {
        let a = prepared
            .propose(
                EvolutionRequest {
                    start: point(i, -2),
                    end: point(i + 1, -2),
                    initial,
                },
                Default::default(),
                &mut b(),
            )
            .unwrap()
            .certify(&mut b());
        certified(&a);
        assert_eq!(a.proof().identity(), &normalized.identity());
        let t = clocked.clock().start() + clocked.clock().duration() * q(i + 1, 4);
        // x(t)=t²-a²+3(t-a), with physical a=1/3, independently of RK stages.
        let exact = &t * &t - q(1, 9) + Rational::from(3) * (&t - q(1, 3));
        initial = a.proof().end_enclosure().unwrap().clone();
        contains(&initial.lanes().unwrap()[0], &exact);
    }
    let next = source
        .on_time_span(q(2, 3), q(7, 6), &mut b())
        .unwrap()
        .integrate(
            initial,
            EvolutionOptions {
                maximum_step: Some(point(1, -2)),
                ..Default::default()
            },
            &mut b(),
        );
    assert!(
        matches!(next.status(), ClockStatus::Completed),
        "{:?}",
        next.status()
    );
    assert_eq!(next.reached_time(), &q(7, 6));
    contains(
        &next.end_enclosure().unwrap().lanes().unwrap()[0],
        &q(15, 4),
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_recomputes_proposals_and_exact_certificates() {
    fn trace(t: &Rk4Trace<1>) -> serde_json::Value {
        serde_json::json!([
            [
                t.options().relative_padding.to_bits(),
                t.options().absolute_padding.to_bits()
            ],
            t.stages()
                .iter()
                .map(|s| (
                    s.time().to_bits(),
                    s.state().map(f64::to_bits),
                    s.derivative().map(|d| d.map(f64::to_bits))
                ))
                .collect::<Vec<_>>(),
            t.end_estimate().map(|v| v.map(f64::to_bits)),
            t.bounds()
                .iter()
                .map(|v| v.estimates().map(|(a, b)| (a.to_bits(), b.to_bits())))
                .collect::<Vec<_>>()
        ])
    }
    type Replay = (
        FieldDefinition,
        EvolutionRequest<1>,
        Rk4Options,
        EnclosurePolicy,
    );
    fn run((definition, request, options, policy): Replay) -> serde_json::Value {
        let mut budget = EnclosureBudget::new(policy);
        let source = ExpressionField::<1>::from_definition(definition, &mut budget).unwrap();
        let prepared = source.rk4_proposer(&mut budget).unwrap();
        let result = match prepared.propose(request, options, &mut budget) {
            Ok(candidate) => {
                let a = candidate.certify(&mut budget);
                let p = a.proof();
                let e = p.evidence();
                serde_json::json!([
                    "proof",
                    trace(a.trace()),
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
                    format!("{:?}", p.status()),
                    p.end_enclosure(),
                    p.tube()
                ])
            }
            Err(failure) => serde_json::json!([
                "proposal failure",
                trace(failure.trace()),
                failure.proposer().field().identity(),
                failure.request(),
                format!("{:?}", failure.error())
            ]),
        };
        serde_json::json!([
            result,
            budget.remaining_graph_work(),
            budget.remaining_scalar_work()
        ])
    }
    const KEY: &str = "HYPERDYNAMICS_RK_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        println!("RK_REPLAY={}", run(serde_json::from_str(&encoded).unwrap()));
        return;
    }
    for (expression, req, options) in [
        (
            Expr::anonymous_symbol(X),
            request(point(1, -2), [d(1, 1)]),
            Rk4Options::default(),
        ),
        (
            Expr::anonymous_symbol(X).powi(2),
            request(point(1, -3), [d(1, 1)]),
            Rk4Options::default(),
        ),
        (
            Expr::int(2) * Expr::anonymous_symbol(X),
            request(d(1, 1), [d(1, 1)]),
            Rk4Options::default(),
        ),
        (
            Expr::int(1) / Expr::anonymous_symbol(X),
            request(point(1, -2), [d(0, 0)]),
            Rk4Options::default(),
        ),
        (
            Expr::int(1),
            request(d(1, 1), [d(0, 0)]),
            Rk4Options {
                relative_padding: -1.0,
                absolute_padding: 0.0,
            },
        ),
    ] {
        let source = field::<1>(&[expression]);
        let definition: FieldDefinition =
            serde_json::from_value(serde_json::to_value(source.identity()).unwrap()).unwrap();
        let input = (
            definition.clone(),
            req.clone(),
            options,
            EnclosurePolicy::default(),
        );
        let expected = run(input.clone());
        let encoded = serde_json::to_string(&input).unwrap();
        let child = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_recomputes_proposals_and_exact_certificates",
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
        let actual: serde_json::Value = serde_json::from_str(
            stdout
                .lines()
                .find_map(|l| l.strip_prefix("RK_REPLAY="))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(actual, expected);
        let mut invalid = definition.clone();
        invalid.state[0] = invalid.time;
        assert!(ExpressionField::<1>::from_definition(invalid, &mut b()).is_err());
        let mut invalid = definition;
        invalid.source.outputs[0] = usize::MAX;
        assert!(ExpressionField::<1>::from_definition(invalid, &mut b()).is_err());
        // A serialized success flag never replaces rebuilding and verifying source data.
        let mut forged = serde_json::to_value(input).unwrap();
        forged[0]["certified"] = serde_json::json!(true);
        forged[0]["candidate"] = serde_json::json!([0, 0]);
        assert_eq!(run(serde_json::from_value(forged).unwrap()), expected);
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
