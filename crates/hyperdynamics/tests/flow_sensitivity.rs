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
    assert_eq!(result.source().reached_time(), stopped.reached_time());

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
            result.source().status(),
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

// Rotation x' = -y, y' = x has the exact derivative [[cos t, -sin t], [sin t, cos t]]
// in source coordinates, whatever chart, frame or proof method produced it.
#[cfg(any(feature = "taylor-evolution", feature = "coordinates"))]
fn rotation() -> [Expr; 2] {
    [-s(1), s(0)]
}
#[cfg(any(feature = "taylor-evolution", feature = "coordinates"))]
fn contains_rotation(m: &IntervalMatrix<DyadicInterval, 2, 2>, t: &Rational, bound: Rational) {
    let [(cl, ch), (sl, sh)] = cos_sin_bounds(t);
    let m = entries(m);
    included(&between(&cl, &ch), &m[0][0]);
    included(&between(&(-&sh), &(-&sl)), &m[0][1]);
    included(&between(&sl, &sh), &m[1][0]);
    included(&between(&cl, &ch), &m[1][1]);
    for x in m.iter().flatten() {
        small(x, bound.clone());
    }
}
#[cfg(any(feature = "taylor-evolution", feature = "coordinates"))]
fn half() -> Rational {
    Rational::from(1) / Rational::from(2)
}
// Riccati x' = x², x(0) = 1/2: ∂x(t)/∂x0 = 1/(1 - t/2)², which is 16/9 at t = 1/2.
#[cfg(feature = "taylor-evolution")]
fn riccati_end(m: &IntervalMatrix<DyadicInterval, 1, 1>, bound: Rational) {
    included(
        &enclosure(&(Rational::from(16) / Rational::from(9))),
        &entries(m)[0][0],
    );
    small(&entries(m)[0][0], bound);
}

#[cfg(feature = "taylor-evolution")]
#[test]
fn taylor_and_adaptive_sources_reuse_their_picard_proofs() {
    use hyperdynamics::field::taylor_flow::evolution::{
        FlowTaylorEvolutionFailure, FlowTaylorEvolutionStatus,
    };
    let f = field::<2>(&rotation());
    let program = f.flow_taylor(4, &mut b()).unwrap();
    let source = Arc::new(program.integrate(
        request(scaled(1, 1, -1), [scaled(-1, 1, -4), scaled(-1, 1, -4)]),
        step(-3),
        &mut b(),
    ));
    assert!(matches!(
        source.status(),
        FlowTaylorEvolutionStatus::Completed
    ));
    let result = source
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert!(Arc::ptr_eq(result.source(), &source));
    assert_eq!(result.steps().len(), source.segments().len());
    contains_rotation(
        result.end_matrix().unwrap(),
        &half(),
        Rational::from(1) / Rational::from(1_000_000),
    );

    // Nonlinear Riccati. Jacobians cover the coarse a-priori Picard tubes, not
    // the tighter Taylor tubes, which bounds the achievable width.
    let riccati = field::<1>(&[s(0) * s(0)]);
    let program = riccati.flow_taylor(3, &mut b()).unwrap();
    let source = Arc::new(program.integrate(
        request(scaled(1, 1, -1), [scaled(1, 1, -1)]),
        step(-3),
        &mut b(),
    ));
    riccati_end(
        source
            .sensitivity(SensitivityOptions::default(), &mut b())
            .unwrap()
            .end_matrix()
            .unwrap(),
        Rational::from(1) / Rational::from(4),
    );

    // A stopped Taylor run keeps its prefix, or reports its own failure.
    let stopped = Arc::new(program.integrate(
        request(scaled(1, 1, -1), [scaled(1, 1, -1)]),
        EvolutionOptions {
            maximum_steps: 2,
            ..step(-4)
        },
        &mut b(),
    ));
    let prefix = stopped
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert_eq!(prefix.steps().len(), 2);
    assert!(prefix.end_matrix().is_none());
    let none =
        Arc::new(program.integrate(request(d(-1, -1), [scaled(1, 1, -1)]), step(-4), &mut b()));
    assert!(matches!(
        none.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(
            FlowTaylorEvolutionFailure::Search(EvolutionFailure::ReversedTime)
        ))
    ));

    #[cfg(feature = "adaptive-taylor")]
    {
        use hyperdynamics::field::taylor_flow::adaptive::{
            AdaptiveTaylorOptions, AdaptiveTaylorStatus,
        };
        let controller = riccati.adaptive_taylor(AdaptiveTaylorOptions {
            maximum_order: 4,
            remainder_limits: [scaled(1, 1, -12)],
            native: step(-3),
            ..Default::default()
        });
        let source =
            Arc::new(controller.integrate(request(scaled(1, 1, -1), [scaled(1, 1, -1)]), &mut b()));
        assert!(matches!(source.status(), AdaptiveTaylorStatus::Completed));
        let result = source
            .sensitivity(SensitivityOptions::default(), &mut b())
            .unwrap();
        assert_eq!(result.steps().len(), source.segments().len());
        riccati_end(
            result.end_matrix().unwrap(),
            Rational::from(1) / Rational::from(4),
        );
    }
}

// x = c + A y with c = (1/4, -1/8) and the exact inverse B = A⁻¹.
#[cfg(feature = "coordinates")]
fn charted(
    f: &ExpressionField<2>,
    a: [[i64; 2]; 2],
    inverse: [[(i64, i64); 2]; 2],
) -> hyperdynamics::field::coordinates::CoordinateField<2> {
    use hyperreal::Real;
    let r = |(n, d): (i64, i64)| Real::from(Rational::from(n) / Rational::from(d));
    f.in_coordinates(
        hypersolve::expression_program::CoordinateChange {
            variables: vec![SymbolId(0), SymbolId(1)],
            to_source: a
                .iter()
                .map(|row| row.iter().map(|&x| r((x, 1))).collect())
                .collect(),
            from_source: inverse
                .iter()
                .map(|row| row.iter().map(|&x| r(x)).collect())
                .collect(),
            origin: vec![r((1, 4)), r((-1, 8))],
        },
        &mut b(),
    )
    .unwrap()
}
// A = [[1, 1], [0, 2]].
#[cfg(feature = "coordinates")]
fn sheared(f: &ExpressionField<2>) -> hyperdynamics::field::coordinates::CoordinateField<2> {
    charted(f, [[1, 1], [0, 2]], [[(1, 1), (-1, 2)], [(0, 1), (1, 2)]])
}
// A = [[1, 0], [1, 1]].
#[cfg(feature = "qr-evolution")]
fn lower(f: &ExpressionField<2>) -> hyperdynamics::field::coordinates::CoordinateField<2> {
    charted(f, [[1, 0], [1, 1]], [[(1, 1), (0, 1)], [(-1, 1), (1, 1)]])
}

#[cfg(feature = "coordinates")]
#[test]
fn fixed_chart_conjugates_chart_derivatives_into_source_coordinates() {
    let f = field::<2>(&rotation());
    let chart = sheared(&f);
    let source = Arc::new(chart.integrate(
        request(scaled(1, 1, -1), [scaled(-1, 1, -4), scaled(-1, 1, -4)]),
        step(-3),
        &mut b(),
    ));
    completed(source.native());
    let result = source
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert_eq!(result.steps().len(), 4);
    let end = result.end_matrix().unwrap();
    contains_rotation(end, &half(), Rational::from(1) / Rational::from(1_000_000));
    // The chart-coordinate derivative B·R·A has (1, 0) entry sin(t)/2. Its
    // exclusion shows the published matrix is conjugated, and in this order.
    let [_, (sl, _)] = cos_sin_bounds(&half());
    excluded(&(&sl / Rational::from(2)), &entries(end)[1][0]);
    // Every tube contains its segment's start and end derivatives.
    let mut previous = IntervalMatrix::new([[d(1, 1), d(0, 0)], [d(0, 0), d(1, 1)]]).unwrap();
    for step in result.steps() {
        for (outer, inner) in [
            (step.global_tube(), &previous),
            (step.global_tube(), step.global_end()),
        ] {
            for (a, b) in entries(inner)
                .iter()
                .flatten()
                .zip(entries(outer).iter().flatten())
            {
                included(a, b);
            }
        }
        previous = step.global_end().clone();
    }

    // The native prefix and native failures decide the claim.
    let stopped = Arc::new(chart.integrate(
        request(scaled(1, 1, -1), [scaled(-1, 1, -4), scaled(-1, 1, -4)]),
        EvolutionOptions {
            maximum_steps: 2,
            ..step(-3)
        },
        &mut b(),
    ));
    let prefix = stopped
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert_eq!(prefix.steps().len(), 2);
    assert!(prefix.end_matrix().is_none());
    assert_eq!(
        prefix.prefix_end_matrix(),
        Some(result.steps()[1].global_end())
    );
    let reversed = Arc::new(chart.integrate(
        request(d(-1, -1), [d(0, 0), d(0, 0)]),
        EvolutionOptions::default(),
        &mut b(),
    ));
    assert!(matches!(
        reversed.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(EvolutionFailure::ReversedTime))
    ));
    let identity = Arc::new(chart.integrate(
        request(d(0, 0), [d(-1, 1), d(-1, 1)]),
        EvolutionOptions::default(),
        &mut b(),
    ));
    let unit = IntervalMatrix::new([[d(1, 1), d(0, 0)], [d(0, 0), d(1, 1)]]).unwrap();
    assert_eq!(
        identity
            .sensitivity(SensitivityOptions::default(), &mut b())
            .unwrap()
            .end_matrix(),
        Some(&unit)
    );
}

#[cfg(feature = "qr-evolution")]
fn qr_options(
    taylor_order: Option<usize>,
    maximum_windows: usize,
) -> hyperdynamics::field::coordinates::qr_evolution::QrEvolutionOptions {
    hyperdynamics::field::coordinates::qr_evolution::QrEvolutionOptions {
        window: scaled(1, 1, -3),
        maximum_windows,
        taylor_order,
        native: step(-4),
    }
}

#[cfg(feature = "qr-evolution")]
#[test]
fn qr_frames_compose_exact_transitions_with_chart_derivatives() {
    use QrFrameChoice::{Proposed, Retained};
    use hyperdynamics::field::coordinates::qr_evolution::{
        QrEvolutionFailure, QrEvolutionStatus, QrFrameChoice, QrWindowEvolution,
    };
    type Chart = fn(&ExpressionField<2>) -> hyperdynamics::field::coordinates::CoordinateField<2>;
    type Reference = fn(&Rational) -> [[DyadicInterval; 2]; 2];
    let initial = [scaled(-1, 1, -4), scaled(-1, 1, -4)];
    let tight = || Rational::from(1) / Rational::from(100_000);
    // Integrates in QR frames, then checks every step's endpoint derivative
    // against the exact reference at that segment's end, and that every tube
    // spans its start and end derivatives.
    let derive = |outputs: &[Expr],
                  chart: Chart,
                  order,
                  windows,
                  initial: [DyadicInterval; 2],
                  reference: Option<Reference>| {
        let source = Arc::new(chart(&field::<2>(outputs)).integrate_qr(
            request(scaled(1, 1, -1), initial),
            qr_options(order, windows),
            &mut b(),
        ));
        let completed = source.completed_windows();
        let times: Vec<Rational> = completed
            .iter()
            .flat_map(|w| match w.evolution().unwrap() {
                QrWindowEvolution::Picard(r) => r
                    .segments()
                    .map(|s| rational_point(&s.proposal().end))
                    .collect::<Vec<_>>(),
                QrWindowEvolution::Taylor(r) => r
                    .segments()
                    .map(|s| rational_point(&s.picard().proposal().end))
                    .collect(),
            })
            .collect();
        let choices: Vec<_> = completed
            .iter()
            .map(|w| w.comparison().unwrap().choice())
            .collect();
        let result = source
            .sensitivity(SensitivityOptions::default(), &mut b())
            .unwrap();
        assert_eq!(result.steps().len(), times.len());
        let mut previous = IntervalMatrix::new([[d(1, 1), d(0, 0)], [d(0, 0), d(1, 1)]]).unwrap();
        for (step, t) in result.steps().iter().zip(&times) {
            let end = entries(step.global_end());
            let tube = entries(step.global_tube());
            for i in 0..2 {
                for j in 0..2 {
                    if let Some(reference) = reference {
                        included(&reference(t)[i][j], &end[i][j]);
                        small(&end[i][j], tight());
                    }
                    included(&end[i][j], &tube[i][j]);
                    included(&entries(&previous)[i][j], &tube[i][j]);
                }
            }
            previous = step.global_end().clone();
        }
        (source, result, choices)
    };
    fn points(m: [[Rational; 2]; 2]) -> [[DyadicInterval; 2]; 2] {
        m.map(|row| row.map(|x| enclosure(&x)))
    }
    let shear: Reference = |t| {
        points([
            [Rational::from(1), Rational::from(4) * t],
            [Rational::from(0), Rational::from(1)],
        ])
    };
    let saddle: Reference = |t| {
        let (lo, hi) = exp_bounds(t);
        let inverse = between(&(Rational::from(1) / &hi), &(Rational::from(1) / &lo));
        [[between(&lo, &hi), d(0, 0)], [d(0, 0), inverse]]
    };
    let turn: Reference = |t| {
        let [(cl, ch), (sl, sh)] = cos_sin_bounds(t);
        [
            [between(&cl, &ch), between(&(-&sh), &(-&sl))],
            [between(&sl, &sh), between(&cl, &ch)],
        ]
    };
    // x' = 4y, y' = -x: [[cos 2t, 2 sin 2t], [-sin(2t)/2, cos 2t]].
    let ellipse: Reference = |t| {
        let [(cl, ch), (sl, sh)] = cos_sin_bounds(&(Rational::from(2) * t));
        let two = Rational::from(2);
        [
            [between(&cl, &ch), between(&(&two * &sl), &(&two * &sh))],
            [between(&(-&sh / &two), &(-&sl / &two)), between(&cl, &ch)],
        ]
    };
    let rotation_field = rotation();
    // Field, chart, Taylor order, window limit, reference and expected choices.
    type Case<'a> = (
        &'a [Expr],
        Chart,
        Option<usize>,
        usize,
        Reference,
        &'a [QrFrameChoice],
    );
    let cases: [Case; 9] = [
        // Every window changes frame.
        (
            &[s(1) * Expr::int(4), Expr::int(0)],
            sheared,
            Some(3),
            8,
            shear,
            &[Proposed; 4],
        ),
        (
            &[s(1) * Expr::int(4), Expr::int(0)],
            sheared,
            None,
            8,
            shear,
            &[Proposed; 4],
        ),
        (
            &[s(0), -s(1)],
            sheared,
            Some(3),
            8,
            saddle,
            &[Proposed, Retained, Retained, Retained],
        ),
        (&[s(0), -s(1)], sheared, None, 8, saddle, &[Proposed; 4]),
        // Frames never change; conjugation alone carries the chart.
        (&rotation_field, sheared, Some(3), 8, turn, &[Retained; 4]),
        (&rotation_field, sheared, None, 8, turn, &[Retained; 4]),
        (
            &rotation_field,
            lower,
            None,
            8,
            turn,
            &[Proposed, Retained, Retained, Retained],
        ),
        (
            &[s(1) * Expr::int(4), -s(0)],
            lower,
            Some(3),
            8,
            ellipse,
            &[Proposed, Retained, Retained, Retained],
        ),
        // A rotating second transition meets a nonidentity chain, so the
        // order of the transition product matters.
        (
            &[s(1) * Expr::int(4), -s(0)],
            lower,
            None,
            2,
            ellipse,
            &[Proposed, Proposed],
        ),
    ];
    for (outputs, chart, order, windows, reference, expected) in cases {
        let (source, result, choices) = derive(
            outputs,
            chart,
            order,
            windows,
            initial.clone(),
            Some(reference),
        );
        assert_eq!(choices, expected, "{:?}", source.status());
        if windows == 8 {
            assert!(
                matches!(source.status(), QrEvolutionStatus::Completed),
                "{:?}",
                source.status()
            );
            assert!(result.end_matrix().is_some());
        } else {
            assert!(result.end_matrix().is_none());
        }
    }
    // The chart-coordinate derivative B·R·A has (1, 0) entry sin(t)/2. Its
    // exclusion shows the published matrix is conjugated, and in this order.
    let (_, result, _) = derive(&rotation_field, sheared, Some(3), 8, initial.clone(), None);
    let [_, (sl, _)] = cos_sin_bounds(&half());
    excluded(
        &(&sl / Rational::from(2)),
        &entries(result.end_matrix().unwrap())[1][0],
    );
    // The pendulum mixes choices; its flow preserves area, so det = 1.
    let (_, result, choices) = derive(
        &[s(1), -s(0).sin()],
        sheared,
        Some(3),
        8,
        [scaled(-1, 1, -10), scaled(-1, 1, -10)],
        None,
    );
    assert_eq!(choices, [Retained, Retained, Proposed, Retained]);
    let m = entries(result.end_matrix().unwrap());
    let det = m[0][0]
        .mul(&m[1][1], &mut scalar())
        .unwrap()
        .sub(
            &m[0][1].mul(&m[1][0], &mut scalar()).unwrap(),
            &mut scalar(),
        )
        .unwrap();
    included(&d(1, 1), &det);
    small(&det, Rational::from(1) / Rational::from(16));

    let chart = sheared(&field::<2>(&rotation()));
    // A stopped run keeps exactly its completed windows.
    let one = chart.integrate_qr(
        request(scaled(1, 1, -1), initial.clone()),
        qr_options(Some(3), 1),
        &mut b(),
    );
    assert!(matches!(
        one.status(),
        QrEvolutionStatus::Stopped(QrEvolutionFailure::WindowLimit { limit: 1 })
    ));
    let first = one.completed_windows()[0].source_segments().len();
    let one = Arc::new(one);
    let prefix = one
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert_eq!(prefix.steps().len(), first);
    assert!(prefix.end_matrix().is_none());
    contains_rotation(
        prefix.prefix_end_matrix().unwrap(),
        &(Rational::from(1) / Rational::from(8)),
        Rational::from(1) / Rational::from(100_000),
    );
    let none = Arc::new(chart.integrate_qr(
        request(scaled(1, 1, -1), initial.clone()),
        qr_options(Some(3), 0),
        &mut b(),
    ));
    assert!(matches!(
        none.sensitivity(SensitivityOptions::default(), &mut b()),
        Err(SensitivityError::Source(QrEvolutionFailure::WindowLimit {
            limit: 0
        }))
    ));
    let empty = Arc::new(chart.integrate_qr(
        EvolutionRequest {
            initial: BoxN::empty().unwrap(),
            ..request(scaled(1, 1, -1), initial.clone())
        },
        qr_options(Some(3), 8),
        &mut b(),
    ));
    assert!(matches!(empty.status(), QrEvolutionStatus::Empty));
    let result = empty
        .sensitivity(SensitivityOptions::default(), &mut b())
        .unwrap();
    assert!(result.steps().is_empty());
    assert!(result.end_matrix().unwrap().is_empty());
}

#[cfg(feature = "qr-evolution")]
#[test]
fn qr_sensitivity_exhaustion_is_exact_and_retry_is_unchanged() {
    let f = field::<2>(&rotation());
    let source = Arc::new(sheared(&f).integrate_qr(
        request(scaled(1, 1, -2), [scaled(-1, 1, -4), scaled(-1, 1, -4)]),
        qr_options(Some(3), 8),
        &mut b(),
    ));
    let mut measured = b();
    let reference = format!(
        "{:?}",
        source
            .sensitivity(SensitivityOptions::default(), &mut measured)
            .unwrap()
            .steps()
    );
    let policy = EnclosurePolicy::default();
    let graph = policy.graph_work - measured.remaining_graph_work();
    let native = policy.scalar_work - measured.remaining_scalar_work();
    let run = |policy: EnclosurePolicy| {
        source.sensitivity(
            SensitivityOptions::default(),
            &mut EnclosureBudget::new(policy),
        )
    };
    let exact = |policy| format!("{:?}", run(policy).unwrap().steps());
    assert_eq!(
        exact(EnclosurePolicy {
            graph_work: graph,
            ..policy
        }),
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
    assert_eq!(
        exact(EnclosurePolicy {
            scalar_work: native,
            ..policy
        }),
        reference
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
    assert_eq!(Arc::strong_count(&source), 1);
    assert_eq!(exact(policy), reference);
}

#[cfg(all(feature = "serde", feature = "qr-evolution"))]
#[test]
fn fresh_process_replay_reproduces_qr_sensitivity() {
    use hyperdynamics::field::coordinates::{
        CoordinateField, CoordinateFieldDefinition, qr_evolution::QrEvolutionOptions,
    };
    type Replay = (
        CoordinateFieldDefinition,
        EvolutionRequest<2>,
        QrEvolutionOptions,
        EnclosurePolicy,
        SensitivityOptions,
    );
    fn signature((definition, request, options, policy, terms): Replay) -> String {
        let chart =
            CoordinateField::<2>::from_definition(definition, &mut EnclosureBudget::new(policy))
                .unwrap();
        let source =
            Arc::new(chart.integrate_qr(request, options, &mut EnclosureBudget::new(policy)));
        let result = source
            .sensitivity(terms, &mut EnclosureBudget::new(policy))
            .unwrap();
        format!(
            "{:?} {:?} {:?}",
            source.status(),
            result.end_matrix(),
            result.steps()
        )
    }
    const KEY: &str = "HYPERDYNAMICS_QR_SENSITIVITY_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let replay: Replay = serde_json::from_str(&encoded).unwrap();
        println!("QR_SENSITIVITY_REPLAY={}", signature(replay));
        return;
    }
    let f = field::<2>(&rotation());
    let chart = sheared(&f);
    let replay: Replay = (
        CoordinateFieldDefinition {
            source: definition::<2>(&rotation()),
            change: chart.transformation().change().clone(),
        },
        request(scaled(1, 1, -2), [scaled(-1, 1, -4), scaled(-1, 1, -4)]),
        qr_options(Some(3), 8),
        EnclosurePolicy::default(),
        SensitivityOptions { terms: 6 },
    );
    let expected = signature(replay.clone());
    let encoded = serde_json::to_string(&replay).unwrap();
    let child = std::process::Command::new(crate::replay::binary().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_reproduces_qr_sensitivity",
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
            .find_map(|s| s.strip_prefix("QR_SENSITIVITY_REPLAY="))
            .unwrap(),
        expected
    );
    // A chart whose inverse proposal is wrong is refused before any derivative.
    let mut corrupted = replay.0.clone();
    corrupted.change.from_source[0][1] = 0.into();
    assert!(CoordinateField::<2>::from_definition(corrupted, &mut b()).is_err());
    // A changed horizon is a different derivation, not a replay.
    let mut changed = replay;
    changed.1.end = scaled(1, 1, -3);
    assert_ne!(signature(changed), expected);
}
