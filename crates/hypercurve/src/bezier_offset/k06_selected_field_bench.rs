//! K06: head-to-head of the selected polynomial field circle route against the
//! cusp-semicircle chord kernel on their only common partner family, finite
//! rational lines. Run with:
//!
//! ```text
//! cargo +1.99.1 test --locked --release -p hypercurve --lib k06_ -- --ignored --nocapture
//! ```
//!
//! Every case first checks that both routes publish the same contacts (exact
//! point equality), the same count and the same certainty. Cost is compared
//! only for agreeing cases.

use super::*;
use crate::bezier_algebraic_image::parameter_representation;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

// Per-thread counters keep parallel unit tests from polluting a measurement.
// The allocator only delegates to `System`; no allocation behavior changes.
struct CountingAllocator;
#[global_allocator]
static K06_ALLOCATOR: CountingAllocator = CountingAllocator;
thread_local! {
    static CALLS: Cell<u64> = const { Cell::new(0) };
    static BYTES: Cell<u64> = const { Cell::new(0) };
}
fn count(bytes: usize) {
    let _ = CALLS.try_with(|calls| calls.set(calls.get() + 1));
    let _ = BYTES.try_with(|total| total.set(total.get() + bytes as u64));
}
// SAFETY: every operation forwards unchanged to the system allocator.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        count(size);
        unsafe { System.realloc(pointer, layout, size) }
    }
}
fn allocations() -> (u64, u64) {
    (CALLS.with(Cell::get), BYTES.with(Cell::get))
}

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
use crate::test_support::{decided, decided_ok};

/// `sqrt(1/2)` center, radius 2, cardinal frame: the existing exact-oblique
/// chord fixture (`algebraic_cusp_semicircle_intersects_exact_oblique_chords_exactly`).
fn sqrt_center(policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![-q(1, 2), Real::zero(), Real::one()],
            policy,
        )
        .unwrap(),
    );
    let interval = decided(
        BezierParameterInterval::try_new_with_policy(q(1, 2), Real::one(), policy).unwrap(),
    );
    let parameter = decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy).unwrap(),
    );
    let center = CurvePoint2::from(
        RationalBezierAlgebraicPointImage2::from_retained_expression(
            parameter.clone(),
            parameter_representation(&parameter, policy),
            vec![Real::zero(), Real::one()],
            vec![Real::zero()],
            vec![Real::one()],
            "K06 sqrt(1/2) center",
        ),
    );
    decided(
        BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &center,
            (1, 0),
            Real::from(2_i8),
            false,
            policy,
        )
        .unwrap(),
    )
    .unwrap()
}

/// The first selected cusp of a parallel at distance `distance`.
fn first_cusp(parallel: &BezierParallel2, policy: &CurveContext) -> BezierAlgebraicParameter2 {
    let analysis = decided(
        parallel
            .singularity_analysis_with_policy(&CurveParameterRange2::unit(), policy)
            .unwrap(),
    );
    let BezierParameter2::Algebraic(parameter) = &analysis.parallel_cusps()[0] else {
        panic!("the selected cusp must be algebraic");
    };
    parameter.clone()
}

/// The existing cubic parallel cusp fixture (`general_algebraic_cusp_semicircle`):
/// rational frame, center at the cusp of the distance-1/2 parallel, radius 1/4.
fn cubic_cusp(policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
    let parallel = CubicBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::zero(), Real::from(4_i8)),
        Point2::new(Real::from(4_i8), Real::from(-4_i8)),
        Point2::new(Real::from(4_i8), Real::zero()),
    )
    .parallel_left(q(1, 2))
    .unwrap();
    let parameter = first_cusp(&parallel, policy);
    decided(
        parallel
            .algebraic_cusp_semicircle(&parameter, q(1, 4), true, policy)
            .unwrap(),
    )
    .unwrap()
}

/// Production parallel-normal frame on the inward distance-1 offset of the
/// ellipse quarter `x^2/4 + y^2 = 1` (rational quadratic, weights 1, 1, 2).
/// The center is the offset's cusp, where the curvature radius equals 1.
fn ellipse_cusp(policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
    let ellipse = RationalQuadraticBezier2::try_new(
        Point2::new(Real::from(2_i8), Real::zero()),
        Point2::new(Real::from(2_i8), Real::one()),
        Point2::new(Real::zero(), Real::one()),
        Real::one(),
        Real::one(),
        Real::from(2_i8),
    )
    .unwrap();
    let parallel = ellipse.parallel_left(Real::one()).unwrap();
    let parameter = first_cusp(&parallel, policy);
    decided(
        BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
            parallel,
            BezierParameter2::Algebraic(parameter).into(),
            q(1, 4),
            false,
            policy,
        )
        .unwrap(),
    )
    .unwrap()
}

type Fixture = fn(&CurveContext) -> BezierAlgebraicCuspSemicircle2;

fn coordinate(
    point: &CurvePoint2,
    x: bool,
) -> crate::selected_field::Result<crate::selected_field::Value> {
    let numerator = if x {
        BivariatePolynomial::new(vec![vec![], vec![Real::one()]])
    } else {
        BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]])
    };
    selected_field_query(
        &PointFormSource2::new(
            point.clone(),
            numerator,
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        ),
        &CurveContext::STRICT,
    )
}

fn approximate(point: &CurvePoint2) -> (f64, f64) {
    let refined = |x: bool| {
        let interval =
            crate::selected_field::bounds(&coordinate(point, x).ok()?, 60, &CurveContext::STRICT)
                .ok()?;
        Some((interval.lower.to_f64_lossy()? + interval.upper.to_f64_lossy()?) / 2.0)
    };
    (
        refined(true).unwrap_or(f64::NAN),
        refined(false).unwrap_or(f64::NAN),
    )
}

/// Number of polynomial generators adjoined above the first ancestor that
/// shares authority with `base`, or `None` when no ancestor does (a foreign
/// field was joined in).
fn generators_above(
    field: &crate::selected_field::Field,
    base: &crate::selected_field::Field,
) -> Option<usize> {
    let mut depth = 0;
    let mut field = field;
    loop {
        if field.shares_authority(base) {
            return Some(depth);
        }
        field = field.parent()?;
        depth += 1;
    }
}

/// A finite rational line `y = x/3 + k` for `x0 <= x <= x1`, as both an exact
/// chord and a polynomial branch whose endpoints are the chord endpoints.
struct Line {
    slope: Real,
    intercept: Real,
    x: [Real; 2],
}
impl Line {
    fn new(intercept_over_1024: i64, x_over_64: [i64; 2]) -> Self {
        Self {
            slope: q(1, 3),
            intercept: q(intercept_over_1024, 1024),
            x: x_over_64.map(|x| q(x, 64)),
        }
    }
    fn y(&self, x: &Real) -> Real {
        &self.slope * x + &self.intercept
    }
    fn chord(&self, policy: &CurveContext) -> BezierAlgebraicChord2 {
        decided(
            BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::new(self.x[0].clone(), self.y(&self.x[0]))),
                CurvePoint2::from(Point2::new(self.x[1].clone(), self.y(&self.x[1]))),
                policy,
            )
            .unwrap(),
        )
    }
    fn branch(&self) -> crate::Curve2 {
        // m*x - y + k = 0; coefficients[i][j] multiplies x^i y^j.
        let equation = BivariatePolynomial::new(vec![
            vec![self.intercept.clone(), Real::from(-1_i8)],
            vec![self.slope.clone()],
        ]);
        let ys = [self.y(&self.x[0]), self.y(&self.x[1])];
        let ys = ys.map(|y| y.to_f64_lossy().unwrap());
        let topology = decompose_bivariate_curve(
            &equation,
            &BivariateCurveDomain {
                first: self.x.clone(),
                second: [
                    Real::from(ys[0].min(ys[1]).floor() as i64 - 1),
                    Real::from(ys[0].max(ys[1]).ceil() as i64 + 1),
                ],
            },
            BivariateTopologyConfig::default(),
        )
        .unwrap();
        let set = crate::PolynomialCurveSet2::from_topology(
            topology,
            hypersolve::AlgebraicFiberRootIsolationConfig::default(),
        )
        .unwrap();
        assert_eq!(
            set.curves().len(),
            1,
            "a line crossing the box is one branch"
        );
        let curve = set.curves()[0].clone();
        assert!(
            curve
                .start()
                .coincides_with(&Point2::new(self.x[0].clone(), self.y(&self.x[0])).into())
                .unwrap()
                && curve
                    .end()
                    .coincides_with(&Point2::new(self.x[1].clone(), self.y(&self.x[1])).into())
                    .unwrap(),
            "the branch and the chord must have identical endpoints"
        );
        curve
    }
}

/// A route result, kept separate by outcome kind.
#[derive(Debug)]
enum Outcome {
    Certified(Vec<CurvePoint2>),
    Uncertain(String),
    Failed(String),
}
impl Outcome {
    fn certainty(&self) -> String {
        match self {
            Self::Certified(_) => "certified".into(),
            Self::Uncertain(reason) => format!("uncertain({reason})"),
            Self::Failed(error) => format!("failed({error})"),
        }
    }
}

/// The per-case kernel, through the same `pub(crate)` entry point used by the
/// region Boolean and corner engines.
fn kernel_route(
    circle: &BezierAlgebraicCuspSemicircle2,
    chord: &BezierAlgebraicChord2,
    policy: &CurveContext,
) -> Outcome {
    match circle.chord_intersections(chord, policy) {
        Ok(Classification::Decided(contacts)) => {
            Outcome::Certified(contacts.into_iter().map(|contact| contact.point).collect())
        }
        Ok(Classification::Uncertain(reason)) => Outcome::Uncertain(format!("{reason:?}")),
        Err(error) => Outcome::Failed(format!("{error:?}")),
    }
}

/// The general route intersects the complete circle, then needs the same
/// half-circle selection the kernel owns. Both phases are returned so the
/// selection cost can be reported separately.
fn general_route_sections(
    circle: &BezierAlgebraicCuspSemicircle2,
    branch: &crate::Curve2,
    policy: &CurveContext,
) -> Result<(CurvePoint2, Vec<CurvePoint2>), Outcome> {
    let center = match circle.center_point_evidence(policy) {
        Ok(Classification::Decided(center)) => center,
        Ok(Classification::Uncertain(reason)) => {
            return Err(Outcome::Uncertain(format!("{reason:?}")));
        }
        Err(error) => return Err(Outcome::Failed(format!("{error:?}"))),
    };
    match branch
        .polynomial_branch()
        .expect("polynomial branch")
        .selected_circle_sections(&center, circle.radial_distance(), policy)
    {
        Ok(sections) => Ok((
            center,
            sections.into_iter().map(|(_, point)| point).collect(),
        )),
        Err(error) => Err(Outcome::Failed(format!("{error:?}"))),
    }
}
fn general_route_select(
    circle: &BezierAlgebraicCuspSemicircle2,
    sections: Vec<CurvePoint2>,
    policy: &CurveContext,
) -> Outcome {
    let half = BezierAlgebraicCuspSemicircleFragment2::full(circle.clone(), policy);
    let mut points = Vec::new();
    for point in sections {
        match half.contains_point(&point, policy) {
            Ok(Classification::Decided(true)) => points.push(point),
            Ok(Classification::Decided(false)) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Outcome::Uncertain(format!("{reason:?}"));
            }
            Err(error) => return Outcome::Failed(format!("{error:?}")),
        }
    }
    Outcome::Certified(points)
}

/// Wall-clock limit for the agreement step. Cancellation is cooperative
/// (`hypersolve::work_budget`); a cancelled route publishes no value.
const AGREEMENT_DEADLINE: Duration = Duration::from_secs(120);

trait Cancelled {
    fn cancelled(reason: String) -> Self;
}
impl Cancelled for Outcome {
    fn cancelled(reason: String) -> Self {
        Self::Uncertain(reason)
    }
}
impl<T> Cancelled for Result<T, Outcome> {
    fn cancelled(reason: String) -> Self {
        Err(Outcome::Uncertain(reason))
    }
}

fn with_deadline<T: Cancelled>(work: impl FnOnce() -> T) -> T {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    let token = hypersolve::work_budget::CancellationToken::new();
    let done = Arc::new(AtomicBool::new(false));
    let watchdog = {
        let (token, done) = (token.clone(), done.clone());
        std::thread::spawn(move || {
            let start = Instant::now();
            while !done.load(Ordering::Acquire) {
                if start.elapsed() >= AGREEMENT_DEADLINE {
                    token.cancel();
                    return;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        })
    };
    let outcome = hypersolve::work_budget::with_cancellable_budget(
        hypersolve::work_budget::WorkBudget::UNLIMITED,
        &token,
        work,
    );
    done.store(true, Ordering::Release);
    watchdog.join().unwrap();
    match outcome.result {
        Ok(value) => value,
        Err(exceeded) => T::cancelled(format!(
            "not completed within {}s: {exceeded:?}",
            AGREEMENT_DEADLINE.as_secs()
        )),
    }
}

fn agreement(kernel: &Outcome, general: &Outcome, policy: &CurveContext) -> Result<usize, String> {
    let (Outcome::Certified(kernel), Outcome::Certified(general)) = (kernel, general) else {
        return Err(format!(
            "certainty differs or is not certified: kernel={} general={}",
            kernel.certainty(),
            general.certainty()
        ));
    };
    if kernel.len() != general.len() {
        return Err(format!(
            "count differs: kernel={} general={}",
            kernel.len(),
            general.len()
        ));
    }
    let equal = |a: &CurvePoint2, b: &CurvePoint2| -> Result<bool, String> {
        let public = a
            .coincides_with(b)
            .map_err(|error| format!("coincides_with: {error:?}"))?;
        let local = match a.same_point(b, policy) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => return Err(format!("same_point: {reason:?}")),
        };
        if public != local {
            return Err("coincides_with and same_point disagree".into());
        }
        Ok(public)
    };
    for (first, second, name) in [(kernel, general, "kernel"), (general, kernel, "general")] {
        for point in first {
            let mut matches = 0;
            for other in second {
                matches += usize::from(equal(point, other)?);
            }
            if matches != 1 {
                return Err(format!(
                    "a {name} contact matched {matches} contacts of the other route"
                ));
            }
        }
    }
    Ok(kernel.len())
}

fn measured<T>(work: impl FnOnce() -> T) -> (T, Duration, (u64, u64)) {
    let before = allocations();
    let start = Instant::now();
    let value = std::hint::black_box(work());
    let elapsed = start.elapsed();
    let after = allocations();
    (value, elapsed, (after.0 - before.0, after.1 - before.1))
}

fn spread(mut samples: Vec<Duration>) -> String {
    samples.sort();
    let us = |d: Duration| d.as_secs_f64() * 1e6;
    format!(
        "{:.1} [{:.1}..{:.1}]",
        us(samples[samples.len() / 2]),
        us(samples[0]),
        us(samples[samples.len() - 1])
    )
}
fn median_u64(mut samples: Vec<u64>) -> u64 {
    samples.sort_unstable();
    samples[samples.len() / 2]
}

/// Runs `work`, returning its hypercurve/hypersolve dispatch paths when the
/// `dispatch-trace` feature is enabled.
fn traced<T>(work: impl FnOnce() -> T) -> (T, Vec<String>) {
    #[cfg(feature = "dispatch-trace")]
    {
        hyperreal::dispatch_trace::reset();
        let value = hyperreal::dispatch_trace::with_recording(work);
        (value, trace_lines())
    }
    #[cfg(not(feature = "dispatch-trace"))]
    {
        (work(), Vec::new())
    }
}

#[cfg(feature = "dispatch-trace")]
fn trace_lines() -> Vec<String> {
    let trace = hyperreal::dispatch_trace::take_trace();
    let mut lines: Vec<String> = trace
        .dispatch
        .iter()
        .filter(|entry| entry.layer == "hypercurve" || entry.layer == "hypersolve")
        .map(|entry| {
            format!(
                "{}/{}/{}={}",
                entry.layer, entry.operation, entry.path, entry.count
            )
        })
        .collect();
    lines.sort();
    lines
}

/// Timed repetitions; trace builds only need the agreement step and traces.
const REPETITIONS: usize = if cfg!(feature = "dispatch-trace") {
    1
} else {
    9
};

#[test]
#[ignore = "K06 benchmark: run in release with --ignored --nocapture"]
fn k06_selected_field_route_versus_cusp_chord_kernel() {
    // Lines have slope 1/3; intercepts are in 1/1024 and x ranges in 1/64.
    // "center" passes within 1e-3 of the circle center; "above"/"below" are
    // shifted vertically by ~0.6 r. Centers (from 60-step field enclosures):
    // sqrt-center (0.70711, 0) r=2 upper half; cubic-cusp (3.54969, -0.65526)
    // r=1/4; ellipse-cusp (1.08178, -0.25992) r=1/4.
    let cases: [(&str, Fixture, &str, i64, [i64; 2]); 9] = [
        ("sqrt-center", sqrt_center, "center", -241, [-338, 430]),
        ("sqrt-center", sqrt_center, "above", 1024, [-338, 430]),
        ("sqrt-center", sqrt_center, "below", -1024, [-338, 430]),
        ("cubic-cusp", cubic_cusp, "center", -1883, [179, 275]),
        ("cubic-cusp", cubic_cusp, "above", -1883 + 154, [179, 275]),
        ("cubic-cusp", cubic_cusp, "below", -1883 - 154, [179, 275]),
        ("ellipse-cusp", ellipse_cusp, "center", -635, [21, 117]),
        ("ellipse-cusp", ellipse_cusp, "above", -635 + 154, [21, 117]),
        ("ellipse-cusp", ellipse_cusp, "below", -635 - 154, [21, 117]),
    ];
    for (name, fixture) in [
        ("sqrt-center", sqrt_center as Fixture),
        ("cubic-cusp", cubic_cusp),
        ("ellipse-cusp", ellipse_cusp),
    ] {
        let circle = fixture(&CurveContext::STRICT);
        let center = decided_ok(circle.center_point_evidence(&CurveContext::STRICT));
        let arc = [q(0, 1), q(1, 2), q(1, 1)].map(|t| {
            approximate(&decided(
                circle.point_evidence_at(&t, &CurveContext::STRICT).unwrap(),
            ))
        });
        eprintln!(
            "K06 fixture {name}: center~{:?} start/mid/end~{arc:?}",
            approximate(&center)
        );
    }
    let mut failures = Vec::new();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (fixture_name, fixture, line_name, intercept, x) in cases {
            let line = Line::new(intercept, x);
            let label = format!("{fixture_name}/{line_name}/{policy:?}");

            // Agreement first, on fresh objects.
            // Each route gets the same wall-clock deadline here; a route that
            // does not finish is reported, not timed.
            let (circle, chord) = (fixture(&policy), line.chord(&policy));
            let started = Instant::now();
            let (kernel, kernel_trace) =
                traced(|| with_deadline(|| kernel_route(&circle, &chord, &policy)));
            let kernel_agreement_time = started.elapsed();
            let (circle, branch) = (fixture(&policy), line.branch());
            let started = Instant::now();
            let ((general, center, full_count), general_trace) = traced(|| {
                match with_deadline(|| Ok(general_route_sections(&circle, &branch, &policy)))
                    .and_then(|sections| sections)
                {
                    Ok((center, sections)) => {
                        let full_count = sections.len();
                        (
                            with_deadline(|| general_route_select(&circle, sections, &policy)),
                            Some(center),
                            Some(full_count),
                        )
                    }
                    Err(outcome) => (outcome, None, None),
                }
            });
            let general_agreement_time = started.elapsed();
            eprintln!(
                "K06 {label}: agreement-step wall time kernel={:.3}s general={:.3}s",
                kernel_agreement_time.as_secs_f64(),
                general_agreement_time.as_secs_f64()
            );
            if cfg!(feature = "dispatch-trace") {
                eprintln!("K06 {label} kernel trace: {kernel_trace:?}");
                eprintln!("K06 {label} general trace: {general_trace:?}");
            }
            let count = match agreement(&kernel, &general, &policy) {
                Ok(count) => count,
                Err(reason) => {
                    eprintln!(
                        "K06 {label}: DISAGREE ({reason}); kernel={kernel:?}; general={general:?}"
                    );
                    failures.push(label);
                    continue;
                }
            };

            // Field structure: each general contact field must extend the
            // retained center field, rather than a join with a foreign field.
            let center_field = center
                .as_ref()
                .and_then(|center| coordinate(center, true).ok())
                .map(|value| value.field().clone());
            let general_fields = match (&general, &center_field) {
                (Outcome::Certified(points), Some(center_field)) => points
                    .iter()
                    .map(|point| {
                        coordinate(point, true)
                            .ok()
                            .and_then(|value| generators_above(value.field(), center_field))
                    })
                    .collect::<Vec<_>>(),
                _ => Vec::new(),
            };
            let kernel_importable = match &kernel {
                Outcome::Certified(points) => points
                    .iter()
                    .map(|point| coordinate(point, true).is_ok())
                    .collect::<Vec<_>>(),
                _ => Vec::new(),
            };

            // Cold timings: fresh fixture objects per repetition, construction
            // outside the timed region. Warm repeats the call on the same
            // objects. Slow general cases get fewer repetitions and no warm.
            let slow = general_agreement_time > Duration::from_secs(5);
            let general_repetitions = if slow {
                REPETITIONS.min(3)
            } else {
                REPETITIONS
            };
            let mut kernel_time = Vec::new();
            let mut kernel_warm = Vec::new();
            let mut kernel_alloc = (Vec::new(), Vec::new());
            for _ in 0..REPETITIONS {
                let (circle, chord) = (fixture(&policy), line.chord(&policy));
                let (_, elapsed, alloc) = measured(|| kernel_route(&circle, &chord, &policy));
                kernel_time.push(elapsed);
                kernel_alloc.0.push(alloc.0);
                kernel_alloc.1.push(alloc.1);
                kernel_warm.push(measured(|| kernel_route(&circle, &chord, &policy)).1);
            }
            let mut general_time = Vec::new();
            let mut general_warm = Vec::new();
            let mut select_time = Vec::new();
            let mut general_alloc = (Vec::new(), Vec::new());
            for _ in 0..general_repetitions {
                let (circle, branch) = (fixture(&policy), line.branch());
                let (sections, solve, solve_alloc) =
                    measured(|| general_route_sections(&circle, &branch, &policy));
                let (_, select, select_alloc) =
                    measured(|| general_route_select(&circle, sections.unwrap().1, &policy));
                general_time.push(solve + select);
                select_time.push(select);
                general_alloc.0.push(solve_alloc.0 + select_alloc.0);
                general_alloc.1.push(solve_alloc.1 + select_alloc.1);
                if !slow {
                    general_warm.push(
                        measured(|| {
                            general_route_select(
                                &circle,
                                general_route_sections(&circle, &branch, &policy).unwrap().1,
                                &policy,
                            )
                        })
                        .1,
                    );
                }
            }
            eprintln!(
                "K06 {label}: AGREE count={count} full_circle_sections={full_count:?} certainty={} \
                 | kernel n={} cold us={} warm us={} allocs={} bytes={} \
                 | general n={general_repetitions} cold us={} (half-selection us={}) warm us={} allocs={} bytes={} \
                 | general generators-above-center={general_fields:?} kernel points importable={kernel_importable:?}",
                general.certainty(),
                REPETITIONS,
                spread(kernel_time),
                spread(kernel_warm),
                median_u64(kernel_alloc.0),
                median_u64(kernel_alloc.1),
                spread(general_time),
                spread(select_time),
                if general_warm.is_empty() {
                    "-".into()
                } else {
                    spread(general_warm)
                },
                median_u64(general_alloc.0),
                median_u64(general_alloc.1),
            );
        }
    }
    assert!(failures.is_empty(), "K06 disagreements: {failures:?}");
}

/// The general route imports every substituted coefficient from the circle
/// center separately and joins each import with the first
/// (`PolynomialPoint2::selected_root_candidates`). This reports whether two
/// imports of one retained center share field authority, and what joining
/// them costs, for each fixture.
#[test]
#[ignore = "K06 diagnostic: run in release with --ignored --nocapture"]
fn k06_center_field_imports() {
    for (name, fixture) in [
        ("sqrt-center", sqrt_center as Fixture),
        ("cubic-cusp", cubic_cusp),
        ("ellipse-cusp", ellipse_cusp),
    ] {
        let policy = CurveContext::STRICT;
        let circle = fixture(&policy);
        let center = decided_ok(circle.center_point_evidence(&policy));
        let ((first, second), import, _) = measured(|| {
            (
                coordinate(&center, true).unwrap(),
                coordinate(&center, false).unwrap(),
            )
        });
        let shared = first.field().shares_authority(second.field());
        let (joined, join, (calls, bytes)) =
            measured(|| crate::selected_field::joined(&first, &second, &policy).unwrap());
        eprintln!(
            "K06 center imports {name}: shared_authority={shared} \
             joined_is_first={} import_us={:.1} join_us={:.1} join_allocs={calls} join_bytes={bytes}",
            joined[0].field().shares_authority(first.field()),
            import.as_secs_f64() * 1e6,
            join.as_secs_f64() * 1e6,
        );
    }
}
