#![cfg(feature = "flow-sensitivity")]
//! Certified initial-state sensitivity against independent references.
use std::sync::Arc;

use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, FieldError,
    evolution::{
        EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
    },
    sensitivity::{FlowSensitivity, SensitivityError, SensitivityOptions},
};
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalMatrix};
use hyperreal::Rational;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};

const T: SymbolId = SymbolId(100);

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
fn s(i: u32) -> Expr {
    Expr::anonymous_symbol(SymbolId(i))
}
fn t() -> Expr {
    Expr::anonymous_symbol(T)
}
fn definition<const N: usize>(outputs: &[Expr]) -> FieldDefinition {
    let state: Vec<_> = (0..N as u32).map(SymbolId).collect();
    let mut variables = state.clone();
    variables.push(T);
    let source = ExpressionProgram::compile(
        outputs,
        &variables,
        &mut ExpressionBudget::new(100_000, 10_000_000),
    )
    .unwrap();
    FieldDefinition {
        source: source.definition().clone(),
        time: T,
        state,
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
fn step(e: i32) -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(scaled(1, 1, e)),
        ..Default::default()
    }
}
fn evolve<const N: usize>(
    f: &ExpressionField<N>,
    request: EvolutionRequest<N>,
    options: EvolutionOptions,
) -> Arc<EvolutionReport<N>> {
    Arc::new(f.integrate(request, options, &mut b()))
}
fn rational_point(d: &DyadicInterval) -> Rational {
    let r = d.to_rational(&mut scalar()).unwrap();
    assert!(r.is_singleton());
    r.lower().unwrap().clone()
}
fn enclosure(r: &Rational) -> DyadicInterval {
    DyadicInterval::from_rational(r, -80, &mut scalar()).unwrap()
}
fn between(lo: &Rational, hi: &Rational) -> DyadicInterval {
    enclosure(lo).hull(&enclosure(hi), &mut scalar()).unwrap()
}
fn included(value: &DyadicInterval, container: &DyadicInterval) {
    assert!(
        value.subset_of(container, &mut scalar()).unwrap(),
        "{value:?} not in {container:?}"
    );
}
fn excluded(value: &Rational, container: &DyadicInterval) {
    assert!(
        !enclosure(value)
            .subset_of(container, &mut scalar())
            .unwrap(),
        "{value:?} unexpectedly in {container:?}"
    );
}
fn entries<const N: usize>(m: &IntervalMatrix<DyadicInterval, N, N>) -> &[[DyadicInterval; N]; N] {
    m.rows().expect("nonempty derivative enclosure")
}
fn width(x: &DyadicInterval) -> Rational {
    let r = x.to_rational(&mut scalar()).unwrap();
    r.upper().unwrap() - r.lower().unwrap()
}
fn small(x: &DyadicInterval, bound: Rational) {
    let w = width(x);
    assert!(w < bound, "{x:?} has width {w:?}");
}
fn sensitivity<const N: usize>(source: &Arc<EvolutionReport<N>>) -> FlowSensitivity<N> {
    source
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap()
}
fn completed<const N: usize>(r: &EvolutionReport<N>) {
    assert!(
        matches!(r.status(), EvolutionStatus::Completed),
        "{:?}",
        r.status()
    );
}
fn segment_times<const N: usize>(r: &EvolutionReport<N>) -> Vec<(Rational, Rational)> {
    r.segments()
        .map(|s| {
            (
                rational_point(&s.proposal().start),
                rational_point(&s.proposal().end),
            )
        })
        .collect()
}

// exp(t) on [0, 1]: the degree-24 Taylor sum and a geometric tail bound, since
// successive tail ratios are at most t/26. The bracket is far narrower than any
// certified enclosure, so containing it implies containing exp(t).
fn exp_bounds(t: &Rational) -> (Rational, Rational) {
    let mut term = Rational::from(1);
    let mut sum = term.clone();
    for k in 1..=24 {
        term = &term * t / Rational::from(k);
        sum = &sum + &term;
    }
    let tail = term * t / Rational::from(25) / (Rational::from(1) - t / Rational::from(26));
    let hi = &sum + &tail;
    (sum, hi)
}

// cos(t) and sin(t) on [0, 1], bracketed by consecutive partial sums of their
// alternating series, whose terms decrease there.
fn cos_sin_bounds(t: &Rational) -> [(Rational, Rational); 2] {
    let bracket = |first: Rational, power: u32| {
        let mut term = first;
        let mut sum = term.clone();
        let mut k = power;
        let mut last = sum.clone();
        for _ in 0..12 {
            term = -(&term * t * t) / Rational::from(((k + 1) * (k + 2)) as i64);
            k += 2;
            last = sum.clone();
            sum = &sum + &term;
        }
        if last < sum { (last, sum) } else { (sum, last) }
    };
    [bracket(Rational::from(1), 0), bracket(t.clone(), 1)]
}

#[test]
fn uncertain_shear_composes_the_exact_derivative_and_tube() {
    let f = field::<2>(&[s(1), Expr::int(0)]);
    let source = evolve(
        &f,
        request(scaled(1, 1, -1), [d(-1, 1), d(-1, 1)]),
        step(-3),
    );
    completed(&source);
    let result = sensitivity(&source);
    assert!(Arc::ptr_eq(result.source(), &source));
    assert_eq!(result.steps().len(), 4);
    let expected = IntervalMatrix::new([[d(1, 1), scaled(1, 1, -1)], [d(0, 0), d(1, 1)]]).unwrap();
    assert_eq!(result.end_matrix(), Some(&expected));
    assert_eq!(result.prefix_end_matrix(), Some(&expected));
    for (step, (a, z)) in result.steps().iter().zip(segment_times(&source)) {
        // The second product is exactly zero, so the series ends with no tail.
        assert_eq!(step.terms().len(), 3);
        assert_eq!(step.tail_norm_bound(), &d(0, 0));
        let tube = entries(step.global_tube());
        assert_eq!(tube[0][0], d(1, 1));
        assert_eq!(tube[1][0], d(0, 0));
        assert_eq!(tube[1][1], d(1, 1));
        included(&between(&a, &z), &tube[0][1]);
    }
}

#[test]
fn exponential_riccati_and_oscillator_contain_independent_derivatives() {
    // x' = x: S(t) = exp(t), for every initial state.
    let exponential = field::<1>(&[s(0)]);
    let source = evolve(
        &exponential,
        request(scaled(1, 1, -1), [d(-1, 2)]),
        EvolutionOptions::default(),
    );
    completed(&source);
    let result = sensitivity(&source);
    for (step, (a, z)) in result.steps().iter().zip(segment_times(&source)) {
        let (lo, hi) = exp_bounds(&z);
        let end = &entries(step.global_end())[0][0];
        included(&between(&lo, &hi), end);
        // Exact Jacobian: the only width is the certified series tail.
        small(end, Rational::from(1) / Rational::from(10_000_000_i64));
        included(
            &between(&exp_bounds(&a).0, &hi),
            &entries(step.global_tube())[0][0],
        );
    }

    // x' = x², x0 in [1/2, 1]: S(t, x0) = 1/(1 - x0 t)², at every initial point.
    let riccati = field::<1>(&[s(0).powi(2)]);
    let source = evolve(
        &riccati,
        request(scaled(1, 1, -2), [scaled(1, 2, -1)]),
        step(-5),
    );
    completed(&source);
    let result = sensitivity(&source);
    for (step, (_, z)) in result.steps().iter().zip(segment_times(&source)) {
        for x0 in [
            Rational::from(1) / Rational::from(2),
            Rational::from(3) / Rational::from(4),
            Rational::from(1),
        ] {
            let denominator = Rational::from(1) - &x0 * &z;
            let exact = Rational::from(1) / (&denominator * &denominator);
            included(&enclosure(&exact), &entries(step.global_end())[0][0]);
        }
    }

    // x' = y, y' = -x: S(t) = [[cos t, sin t], [-sin t, cos t]].
    let oscillator = field::<2>(&[s(1), -s(0)]);
    let source = evolve(
        &oscillator,
        request(scaled(1, 1, -1), [d(1, 1), d(-1, 0)]),
        step(-3),
    );
    completed(&source);
    let result = sensitivity(&source);
    for (step, (_, z)) in result.steps().iter().zip(segment_times(&source)) {
        let [(cos_lo, cos_hi), (sin_lo, sin_hi)] = cos_sin_bounds(&z);
        let m = entries(step.global_end());
        let cos = between(&cos_lo, &cos_hi);
        included(&cos, &m[0][0]);
        included(&between(&sin_lo, &sin_hi), &m[0][1]);
        included(&between(&-sin_hi, &-sin_lo), &m[1][0]);
        included(&cos, &m[1][1]);
        for entry in m.iter().flatten() {
            small(entry, Rational::from(1) / Rational::from(1_000_000_000_i64));
        }
    }
}

#[test]
fn noncommuting_time_dependent_field_excludes_the_commuting_shortcut() {
    // x' = x + t y, y' = 0. A(t) = [[1, t], [0, 0]] and A(s)A(t) != A(t)A(s).
    // The exact derivative is S(t) = [[e^t, e^t - t - 1], [0, 1]], while
    // exp(∫A) = [[e^t, t(e^t - 1)/2], [0, 1]] would follow from commutativity.
    let f = field::<2>(&[s(0) + t() * s(1), Expr::int(0)]);
    let source = evolve(&f, request(scaled(1, 1, -1), [d(-1, 1), d(2, 3)]), step(-8));
    completed(&source);
    let result = sensitivity(&source);
    let (z, m) = {
        let (_, z) = segment_times(&source).pop().unwrap();
        (z, entries(result.end_matrix().unwrap()).clone())
    };
    let (lo, hi) = exp_bounds(&z);
    let one = Rational::from(1);
    included(&between(&lo, &hi), &m[0][0]);
    included(&between(&(&lo - &z - &one), &(&hi - &z - &one)), &m[0][1]);
    // Every entry carries the certified tail radius, even structural zeros.
    included(&d(0, 0), &m[1][0]);
    included(&d(1, 1), &m[1][1]);
    for entry in [&m[1][0], &m[1][1]] {
        small(
            entry,
            Rational::from(1) / Rational::from(1_000_000_000_000_i64),
        );
    }
    // Both commuting-shortcut bounds lie outside the certified entry.
    for e in [lo, hi] {
        excluded(&(&z * (e - &one) / Rational::from(2)), &m[0][1]);
    }
}

#[test]
fn truncated_series_and_coarse_grids_still_enclose() {
    let f = field::<1>(&[s(0)]);
    let source = evolve(&f, request(scaled(1, 1, -2), [d(1, 1)]), step(-2));
    completed(&source);
    let (lo, hi) = exp_bounds(&(Rational::from(1) / Rational::from(4)));
    let exact = between(&lo, &hi);
    let mut previous = Rational::from(0);
    for terms in [8, 2, 1, 0] {
        let result = source
            .sensitivity(SensitivityOptions { terms }, &mut b())
            .unwrap();
        let step = &result.steps()[0];
        assert_eq!(step.terms().len(), terms + 1);
        assert_eq!(
            step.tail_norm_bound().signs(),
            hyperinterval::SignSet::ZeroOrPositive
        );
        let entry = &entries(result.end_matrix().unwrap())[0][0];
        included(&exact, entry);
        // Fewer terms never tighten the enclosure.
        let w = width(entry);
        assert!(
            w >= previous,
            "{terms} terms gave {w:?}, below {previous:?}"
        );
        previous = w;
    }
    let coarse = EnclosurePolicy {
        precision: -4,
        ..Default::default()
    };
    for terms in [0, 3] {
        let result = source
            .sensitivity(
                SensitivityOptions { terms },
                &mut EnclosureBudget::new(coarse),
            )
            .unwrap();
        assert_eq!(result.policy(), coarse);
        included(&exact, &entries(result.end_matrix().unwrap())[0][0]);
    }
}

#[test]
fn prefixes_empty_identity_and_invalid_requests_keep_their_status() {
    // x' = x² from x0 = 1 blows up at t = 1; three steps certify only a prefix.
    let riccati = field::<1>(&[s(0).powi(2)]);
    let stopped = evolve(
        &riccati,
        request(d(2, 2), [d(1, 1)]),
        EvolutionOptions {
            maximum_steps: 3,
            ..Default::default()
        },
    );
    assert!(matches!(
        stopped.status(),
        EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 3 })
    ));
    let result = sensitivity(&stopped);
    assert_eq!(result.steps().len(), 3);
    assert!(result.end_matrix().is_none());
    let (_, z) = segment_times(&stopped).pop().unwrap();
    let denominator = Rational::from(1) - &z;
    included(
        &enclosure(&(Rational::from(1) / (&denominator * &denominator))),
        &entries(result.prefix_end_matrix().unwrap())[0][0],
    );
    assert_eq!(result.reached_time(), stopped.reached_time());

    // Stopped with no certified segment, and invalid requests, are source errors.
    let f = field::<1>(&[s(0)]);
    let none = evolve(
        &f,
        request(d(1, 1), [d(1, 1)]),
        EvolutionOptions {
            maximum_steps: 0,
            ..Default::default()
        },
    );
    assert!(matches!(
        none.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(EvolutionFailure::StepLimit {
            limit: 0
        }))
    ));
    let reversed = evolve(
        &f,
        request(d(-1, -1), [d(1, 1)]),
        EvolutionOptions::default(),
    );
    assert!(matches!(
        reversed.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(EvolutionFailure::ReversedTime))
    ));
    // An empty initial set does not excuse an invalid request.
    let empty_box = BoxN::<DyadicInterval, 1>::empty().unwrap();
    let empty_reversed = evolve(
        &f,
        EvolutionRequest {
            initial: empty_box.clone(),
            ..request(d(-1, -1), [d(0, 0)])
        },
        EvolutionOptions::default(),
    );
    assert!(matches!(
        empty_reversed.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(EvolutionFailure::ReversedTime))
    ));
    let nonpoint = evolve(
        &f,
        EvolutionRequest {
            initial: empty_box.clone(),
            ..request(d(1, 2), [d(0, 0)])
        },
        EvolutionOptions::default(),
    );
    assert!(matches!(
        nonpoint.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(EvolutionFailure::NonPointTime))
    ));

    // A valid empty request has the empty derivative set.
    let empty = evolve(
        &f,
        EvolutionRequest {
            initial: empty_box,
            ..request(d(1, 1), [d(0, 0)])
        },
        EvolutionOptions::default(),
    );
    assert!(matches!(empty.status(), EvolutionStatus::Empty));
    let result = sensitivity(&empty);
    assert!(result.steps().is_empty());
    assert!(result.end_matrix().unwrap().is_empty());

    // A zero-duration request has the exact identity derivative.
    let identity = evolve(
        &f,
        request(d(0, 0), [d(-3, 5)]),
        EvolutionOptions::default(),
    );
    assert!(matches!(identity.status(), EvolutionStatus::Identity));
    let result = sensitivity(&identity);
    assert!(result.steps().is_empty());
    assert_eq!(
        result.end_matrix(),
        Some(&IntervalMatrix::new([[d(1, 1)]]).unwrap())
    );

    // Zero dimensions: the unit derivative, with no series terms.
    let unit = field::<0>(&[]);
    let source = evolve(&unit, request(d(1, 1), []), EvolutionOptions::default());
    completed(&source);
    let result = sensitivity(&source);
    assert!(!result.steps().is_empty());
    assert!(result.steps().iter().all(|s| s.terms().len() == 1));
    assert_eq!(result.end_matrix(), Some(&IntervalMatrix::new([]).unwrap()));
}

#[test]
fn exhaustion_and_cancellation_publish_nothing_and_allow_retry() {
    let f = field::<2>(&[s(1), -s(0)]);
    let source = evolve(&f, request(scaled(1, 1, -1), [d(1, 1), d(0, 0)]), step(-3));
    completed(&source);
    let reference = format!("{:?}", sensitivity(&source).steps());

    // Exact work accounting: the measured amount suffices and one unit less fails.
    let mut measured = b();
    source
        .sensitivity(SensitivityOptions::default(), &mut measured)
        .unwrap();
    let policy = EnclosurePolicy::default();
    let graph = policy.graph_work - measured.remaining_graph_work();
    let native = policy.scalar_work - measured.remaining_scalar_work();
    let run = |policy: EnclosurePolicy| {
        source.sensitivity(
            SensitivityOptions::default(),
            &mut EnclosureBudget::new(policy),
        )
    };
    assert_eq!(
        format!(
            "{:?}",
            run(EnclosurePolicy {
                graph_work: graph,
                ..policy
            })
            .unwrap()
            .steps()
        ),
        reference
    );
    assert!(matches!(
        run(EnclosurePolicy {
            graph_work: graph - 1,
            ..policy
        }),
        Err(SensitivityError::Field(FieldError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(
        run(EnclosurePolicy {
            scalar_work: native,
            ..policy
        })
        .is_ok()
    );
    assert!(matches!(
        run(EnclosurePolicy {
            scalar_work: native - 1,
            ..policy
        }),
        Err(SensitivityError::Field(FieldError::Scalar(
            IntervalError::WorkExhausted
        )))
    ));
    assert!(matches!(
        run(EnclosurePolicy {
            graph_entries: 8,
            ..policy
        }),
        Err(SensitivityError::Field(FieldError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    assert!(matches!(
        run(EnclosurePolicy {
            scalar_bits: 4,
            ..policy
        }),
        Err(SensitivityError::Field(FieldError::Scalar(_)))
    ));

    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = b();
    cancelled.set_cancellation(Some(token));
    assert!(matches!(
        source.sensitivity(SensitivityOptions::default(), &mut cancelled),
        Err(SensitivityError::Field(FieldError::Expression(
            ExpressionError::Cancelled
        )))
    ));

    // Failures leave only the caller's handle, and a fresh budget retries.
    assert_eq!(Arc::strong_count(&source), 1);
    assert_eq!(format!("{:?}", sensitivity(&source).steps()), reference);
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn sensitivity_dispatches_no_scalar_kernels() {
    use hyperreal::dispatch_trace;
    let f = field::<2>(&[s(1), -s(0).sin()]);
    let source = evolve(
        &f,
        request(scaled(1, 1, -2), [scaled(1, 2, -2), d(0, 0)]),
        EvolutionOptions::default(),
    );
    completed(&source);
    dispatch_trace::reset();
    let result = dispatch_trace::with_recording(|| sensitivity(&source));
    let trace = dispatch_trace::take_trace();
    assert!(result.end_matrix().is_some());
    assert!(trace.dispatch.is_empty());
    assert_eq!(
        trace.rational,
        dispatch_trace::RationalTraceStats::default()
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_reproduces_sensitivity_and_rejects_corrupted_controls() {
    type Replay = (
        FieldDefinition,
        EvolutionRequest<2>,
        EvolutionOptions,
        EnclosurePolicy,
        SensitivityOptions,
    );
    fn signature(f: &ExpressionField<2>, (_, request, options, policy, terms): Replay) -> String {
        let source = Arc::new(f.integrate(request, options, &mut EnclosureBudget::new(policy)));
        let result = source
            .sensitivity(terms, &mut EnclosureBudget::new(policy))
            .unwrap();
        format!(
            "{:?} {:?} {:?}",
            result.status(),
            result.end_matrix(),
            result.steps()
        )
    }
    const KEY: &str = "HYPERDYNAMICS_SENSITIVITY_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let replay: Replay = serde_json::from_str(&encoded).unwrap();
        let f = ExpressionField::<2>::from_definition(replay.0.clone(), &mut b()).unwrap();
        println!("SENSITIVITY_REPLAY={}", signature(&f, replay));
        return;
    }
    let definition = definition::<2>(&[s(1), -s(0).sin()]);
    let f = ExpressionField::<2>::from_definition(definition.clone(), &mut b()).unwrap();
    let replay: Replay = (
        definition,
        request(scaled(1, 1, -2), [scaled(1, 2, -2), d(0, 0)]),
        step(-3),
        EnclosurePolicy::default(),
        SensitivityOptions { terms: 5 },
    );
    let expected = signature(&f, replay.clone());
    let encoded = serde_json::to_string(&replay).unwrap();
    let child = std::process::Command::new(crate::replay::binary().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_reproduces_sensitivity_and_rejects_corrupted_controls",
            "--nocapture",
        ])
        .env(KEY, &encoded)
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
            .find_map(|s| s.strip_prefix("SENSITIVITY_REPLAY="))
            .unwrap(),
        expected
    );

    // Corrupted controls are rejected before any derivative is published.
    let mut value: serde_json::Value = serde_json::from_str(&encoded).unwrap();
    value[4]["terms"] = serde_json::json!(-1);
    assert!(serde_json::from_value::<Replay>(value).is_err());
    let mut wrong_state = replay.0.clone();
    wrong_state.state.pop();
    assert!(ExpressionField::<2>::from_definition(wrong_state, &mut b()).is_err());
    let mut wrong_output = replay.0.clone();
    wrong_output.source.outputs.push(usize::MAX);
    assert!(ExpressionField::<2>::from_definition(wrong_output, &mut b()).is_err());
    // A changed horizon is a different derivation, not a replay.
    let mut changed = replay;
    changed.1.end = scaled(1, 1, -3);
    assert_ne!(signature(&f, changed), expected);
}

#[cfg(feature = "serde")]
#[path = "support/replay.rs"]
mod replay;
