#![cfg(feature = "field")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    FieldError,
    clock::*,
    evolution::{EvolutionFailure, EvolutionOptions, EvolutionStatus},
    picard::PicardStatus,
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
fn q(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
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
fn quarter_steps() -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(DyadicInterval::point(1.into(), -2).unwrap()),
        ..Default::default()
    }
}
fn contains(interval: &DyadicInterval, expected: &Rational) {
    assert!(
        interval
            .to_rational(&mut scalar())
            .unwrap()
            .contains_point(expected),
        "{expected:?} outside {interval:?}"
    );
}
fn complete<const N: usize>(r: &ClockEvolution<N>) {
    assert!(
        matches!(r.status(), ClockStatus::Completed),
        "{:?}",
        r.status()
    );
    assert!(matches!(
        r.normalized().status(),
        EvolutionStatus::Completed
    ));
    assert!(r.mapping_error().is_none());
    assert!(r.end_enclosure().is_some());
    assert_eq!(r.reached_time(), r.field().clock().end());
    assert_eq!(r.segments().len(), r.normalized().segments().len());
    let clock = r.field().clock();
    let mut start = clock.start().clone();
    for segment in r.segments() {
        assert_eq!(segment.original_identity(), r.field().original_identity());
        assert_eq!(segment.clock().duration(), clock.duration());
        let proof = segment.normalized_proof();
        assert_eq!(proof.identity(), &r.field().normalized_field().identity());
        assert!(matches!(proof.status(), PicardStatus::Certified));
        assert_eq!(segment.time().lower(), Some(&start));
        let tau = proof.proposal().end.to_rational(&mut scalar()).unwrap();
        let expected = clock.start() + clock.duration() * tau.lower().unwrap();
        assert_eq!(segment.time().upper(), Some(&expected));
        assert!(start < expected);
        assert_eq!(segment.end_enclosure(), proof.end_enclosure().unwrap());
        assert_eq!(segment.tube(), proof.tube().unwrap());
        start = expected;
    }
    assert_eq!(&start, clock.end());
}

#[test]
fn rational_physical_coverage_uncertainty_and_endpoint_composition() {
    for c in -3..=3 {
        for k in -3..=3 {
            let source = field::<1>(&[Expr::int(c)]);
            let f = source.on_time_span(q(1, 3), q(2, 3), &mut b()).unwrap();
            assert_eq!(f.clock().duration(), &q(1, 3));
            assert_eq!(f.original_identity(), &source.identity());
            assert_eq!(
                f.transformation().source().definition(),
                source.identity().source()
            );
            let result = f.integrate(BoxN::new([d(k, k + 1)]).unwrap(), quarter_steps(), &mut b());
            complete(&result);
            assert_eq!(result.segments().len(), 4);
            for (i, segment) in result.segments().enumerate() {
                // The step cap is 1/4 of the chart, hence 1/12 physical time.
                assert_eq!(segment.time().upper(), Some(&q(5 + i as i64, 12)));
                let elapsed = q(i as i64 + 1, 12);
                let end = &segment.end_enclosure().lanes().unwrap()[0];
                contains(end, &(Rational::from(k) + Rational::from(c) * &elapsed));
                contains(end, &(Rational::from(k + 1) + Rational::from(c) * elapsed));
            }
            let next = source
                .on_time_span(q(2, 3), q(7, 6), &mut b())
                .unwrap()
                .integrate(
                    result.end_enclosure().unwrap().clone(),
                    quarter_steps(),
                    &mut b(),
                );
            complete(&next);
            let end = &next.end_enclosure().unwrap().lanes().unwrap()[0];
            contains(end, &(Rational::from(k) + q(5 * c, 6)));
            contains(end, &(Rational::from(k + 1) + q(5 * c, 6)));
        }
    }
}

#[test]
fn physical_time_affine_and_nonlinear_flows_have_independent_controls() {
    let source = field::<1>(&[Expr::int(2) * Expr::anonymous_symbol(T) + Expr::int(3)]);
    for (a, end) in [(q(1, 3), q(2, 3)), (q(-2, 3), q(-1, 5))] {
        let r = source
            .on_time_span(a.clone(), end, &mut b())
            .unwrap()
            .integrate(BoxN::new([d(0, 0)]).unwrap(), quarter_steps(), &mut b());
        complete(&r);
        for segment in r.segments() {
            let t = segment.time().upper().unwrap();
            let expected = t * t - &a * &a + Rational::from(3) * (t - &a);
            contains(&segment.end_enclosure().lanes().unwrap()[0], &expected);
        }
    }
    let source = field::<1>(&[Expr::anonymous_symbol(X).powi(2)]);
    let r = source
        .on_time_span(q(1, 3), q(1, 2), &mut b())
        .unwrap()
        .integrate(BoxN::new([d(1, 1)]).unwrap(), quarter_steps(), &mut b());
    complete(&r);
    for segment in r.segments() {
        let at = |t: &Rational| Rational::one() / (Rational::one() - (t - q(1, 3)));
        contains(
            &segment.end_enclosure().lanes().unwrap()[0],
            &at(segment.time().upper().unwrap()),
        );
        contains(
            &segment.tube().lanes().unwrap()[0],
            &at(segment.time().lower().unwrap()),
        );
        contains(
            &segment.tube().lanes().unwrap()[0],
            &at(segment.time().upper().unwrap()),
        );
    }
    contains(&r.end_enclosure().unwrap().lanes().unwrap()[0], &q(6, 5));
}

#[test]
fn close_and_large_physical_times_are_not_demoted() {
    let power = |n| (0..n).fold(Rational::one(), |a, _| a * Rational::from(2));
    let tiny = Rational::one() / (Rational::from(3) * power(300));
    let large = power(512) + q(1, 3);
    for (a, h) in [(Rational::one(), tiny), (large, q(2, 3))] {
        let end = &a + &h;
        let f = field::<1>(&[Expr::int(1)])
            .on_time_span(a.clone(), end.clone(), &mut b())
            .unwrap();
        assert_eq!(f.clock().duration(), &h);
        assert!(f.clock().duration() > &Rational::zero());
        let r = f.integrate(BoxN::new([d(0, 0)]).unwrap(), quarter_steps(), &mut b());
        complete(&r);
        assert_eq!(r.reached_time(), &end);
        assert_ne!(r.reached_time(), &a);
        contains(&r.end_enclosure().unwrap().lanes().unwrap()[0], &h);
    }
}

#[test]
fn zero_horizon_empty_unit_and_invalid_inputs_keep_distinct_meanings() {
    let source = field::<1>(&[Expr::int(1) / Expr::anonymous_symbol(X)]);
    let identity = source
        .on_time_span(q(1, 3), q(1, 3), &mut b())
        .unwrap()
        .integrate(BoxN::new([d(0, 0)]).unwrap(), Default::default(), &mut b());
    // Identity makes no existence claim for the field at this singular state.
    assert!(matches!(identity.status(), ClockStatus::Identity));
    assert_eq!(identity.end_enclosure().unwrap().lanes(), Some(&[d(0, 0)]));
    assert!(identity.normalized().attempts().is_empty());
    assert_eq!(identity.reached_time(), &q(1, 3));
    assert!(matches!(
        source.on_time_span(q(2, 3), q(1, 3), &mut b()),
        Err(ClockError::ReversedTime)
    ));
    let f = source.on_time_span(q(1, 3), q(2, 3), &mut b()).unwrap();
    let empty = f.integrate(BoxN::empty().unwrap(), Default::default(), &mut b());
    assert!(matches!(empty.status(), ClockStatus::Empty));
    assert!(empty.end_enclosure().unwrap().is_empty());
    assert_eq!(empty.reached_time(), &q(2, 3));
    let invalid = f.integrate(
        BoxN::empty().unwrap(),
        EvolutionOptions {
            maximum_step: Some(d(0, 0)),
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        invalid.status(),
        ClockStatus::Stopped(ClockFailure::Evolution(
            EvolutionFailure::InvalidMaximumStep
        ))
    ));
    let pole = f.integrate(
        BoxN::new([d(0, 0)]).unwrap(),
        EvolutionOptions {
            maximum_halvings: 2,
            ..Default::default()
        },
        &mut b(),
    );
    assert!(matches!(
        pole.status(),
        ClockStatus::Stopped(ClockFailure::Evolution(EvolutionFailure::HalvingLimit {
            limit: 2
        }))
    ));
    assert_eq!(pole.normalized().attempts().len(), 3);
    assert_eq!(pole.segments().len(), 0);
    assert!(pole.end_enclosure().is_none());
    let unit = field::<0>(&[])
        .on_time_span(q(1, 3), q(2, 3), &mut b())
        .unwrap()
        .integrate(BoxN::new([]).unwrap(), Default::default(), &mut b());
    complete(&unit);
    assert!(!unit.end_enclosure().unwrap().is_empty());
}

#[test]
fn construction_and_materialization_share_admission_and_preserve_failure_scope() {
    let source = field::<1>(&[Expr::int(1)]);
    let mut measured = b();
    source
        .on_time_span(q(1, 3), q(2, 3), &mut measured)
        .unwrap();
    let p = measured.policy();
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw,
        scalar_work: sw,
        ..p
    });
    let f = source.on_time_span(q(1, 3), q(2, 3), &mut exact).unwrap();
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    for policy in [
        EnclosurePolicy {
            graph_work: gw - 1,
            ..p
        },
        EnclosurePolicy {
            scalar_work: sw - 1,
            ..p
        },
    ] {
        assert!(
            source
                .on_time_span(q(1, 3), q(2, 3), &mut EnclosureBudget::new(policy))
                .is_err()
        );
    }
    let initial = BoxN::new([d(0, 0)]).unwrap();
    let mut measured = b();
    complete(&f.integrate(initial.clone(), quarter_steps(), &mut measured));
    let gw = p.graph_work - measured.remaining_graph_work();
    let sw = p.scalar_work - measured.remaining_scalar_work();
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: gw,
        scalar_work: sw,
        ..p
    });
    complete(&f.integrate(initial.clone(), quarter_steps(), &mut exact));
    assert_eq!(exact.remaining_graph_work(), 0);
    assert_eq!(exact.remaining_scalar_work(), 0);
    let short = f.integrate(
        initial.clone(),
        quarter_steps(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            scalar_work: sw - 1,
            ..p
        }),
    );
    assert!(matches!(
        short.normalized().status(),
        EvolutionStatus::Completed
    ));
    assert!(matches!(
        short.status(),
        ClockStatus::Stopped(ClockFailure::Mapping(ClockError::Field(
            FieldError::Scalar(IntervalError::WorkExhausted)
        )))
    ));
    assert_eq!(short.segments().len(), 4);
    assert_eq!(short.reached_time(), &q(2, 3));
    assert!(short.prefix_end_enclosure().is_some());
    assert!(short.end_enclosure().is_none());
    let short = f.integrate(
        initial.clone(),
        quarter_steps(),
        &mut EnclosureBudget::new(EnclosurePolicy {
            graph_work: gw - 1,
            ..p
        }),
    );
    assert!(matches!(
        short.normalized().status(),
        EvolutionStatus::Completed
    ));
    assert!(matches!(
        short.status(),
        ClockStatus::Stopped(ClockFailure::Mapping(ClockError::Field(
            FieldError::Expression(ExpressionError::WorkExhausted)
        )))
    ));
    assert_eq!(short.segments().len(), 3);
    assert_eq!(short.reached_time(), &q(7, 12));
    assert!(short.prefix_end_enclosure().is_some());
    assert!(short.end_enclosure().is_none());
    let stopped = f.integrate(
        initial.clone(),
        EvolutionOptions {
            maximum_steps: 1,
            ..quarter_steps()
        },
        &mut b(),
    );
    assert!(matches!(
        stopped.status(),
        ClockStatus::Stopped(ClockFailure::Evolution(EvolutionFailure::StepLimit {
            limit: 1
        }))
    ));
    assert_eq!(stopped.segments().len(), 1);
    assert_eq!(stopped.reached_time(), &q(5, 12));
    assert!(stopped.prefix_end_enclosure().is_some());
    assert!(stopped.end_enclosure().is_none());
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = b();
    cancelled.set_cancellation(Some(token));
    assert!(matches!(
        source.on_time_span(q(1, 3), q(2, 3), &mut cancelled),
        Err(ClockError::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    let result = f.integrate(initial.clone(), quarter_steps(), &mut cancelled);
    assert!(matches!(
        result.status(),
        ClockStatus::Stopped(ClockFailure::Evolution(EvolutionFailure::Field(
            FieldError::Expression(ExpressionError::Cancelled)
        )))
    ));
    assert!(matches!(
        result.mapping_error(),
        Some(ClockError::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert!(result.normalized().attempts().is_empty());
    complete(&f.integrate(initial, quarter_steps(), &mut b()));
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_clock_and_rejects_invalid_source() {
    fn signature(r: &ClockEvolution<1>) -> String {
        complete(r);
        let attempts: Vec<_> = r
            .normalized()
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
                    format!("{:?}", p.status()),
                ])
            })
            .collect();
        serde_json::to_string(&(
            r.field(),
            r.field().clock(),
            r.field().normalized_field().identity(),
            attempts,
            r.segments().map(|s| s.time()).collect::<Vec<_>>(),
            r.reached_time(),
            r.end_enclosure(),
            format!("{:?}", r.status()),
        ))
        .unwrap()
    }
    const KEY: &str = "HYPERDYNAMICS_RATIONAL_TIME_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, initial, options, policy): (
            RationalTimeDefinition,
            BoxN<DyadicInterval, 1>,
            EvolutionOptions,
            EnclosurePolicy,
        ) = serde_json::from_str(&encoded).unwrap();
        let mut budget = EnclosureBudget::new(policy);
        let f = RationalTimeField::<1>::from_definition(definition, &mut budget).unwrap();
        println!(
            "RATIONAL_TIME_REPLAY={}",
            signature(&f.integrate(initial, options, &mut budget))
        );
        return;
    }
    let f = field::<1>(&[Expr::anonymous_symbol(X).powi(2)])
        .on_time_span(q(1, 3), q(1, 2), &mut b())
        .unwrap();
    let initial = BoxN::new([d(1, 1)]).unwrap();
    let original = f.integrate(initial.clone(), quarter_steps(), &mut b());
    let expected = signature(&original);
    let encoded =
        serde_json::to_string(&(&f, &initial, quarter_steps(), EnclosurePolicy::default()))
            .unwrap();
    let child = std::process::Command::new(crate::replay::binary().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_rebuilds_clock_and_rejects_invalid_source",
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
            .find_map(|s| s.strip_prefix("RATIONAL_TIME_REPLAY="))
            .unwrap(),
        expected
    );
    let mut value = serde_json::to_value(&f).unwrap();
    value["duration"] = serde_json::json!(0);
    value["completed"] = serde_json::json!(true);
    let rebuilt =
        RationalTimeField::<1>::from_definition(serde_json::from_value(value).unwrap(), &mut b())
            .unwrap();
    assert_eq!(rebuilt.clock().duration(), &q(1, 6));
    assert_eq!(
        signature(&rebuilt.integrate(initial.clone(), quarter_steps(), &mut b())),
        expected
    );
    let mut changed: RationalTimeDefinition =
        serde_json::from_value(serde_json::to_value(&f).unwrap()).unwrap();
    changed.end = q(5, 12);
    let shorter = RationalTimeField::<1>::from_definition(changed.clone(), &mut b())
        .unwrap()
        .integrate(initial, quarter_steps(), &mut b());
    complete(&shorter);
    assert_eq!(shorter.reached_time(), &q(5, 12));
    contains(
        &shorter.end_enclosure().unwrap().lanes().unwrap()[0],
        &q(12, 11),
    );
    changed.source.source.outputs[0] = usize::MAX;
    assert!(RationalTimeField::<1>::from_definition(changed, &mut b()).is_err());
    assert_eq!(signature(&original), expected);
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
