//! Performance atlas for Hyperdynamics: validated ODE preparation, single-step
//! certification and whole-horizon integration across fields, horizons, Taylor
//! orders, state dimensions, initial-box widths, step sizes, noise-column caps
//! and nonlinearity. Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperdynamics --all-features --example perf_atlas -- --out atlas.json`.
//!
//! Every timed call builds a fresh `EnclosureBudget` with generous work limits
//! and a 2^20-bit per-stage scalar limit, so a failure is a rejected proof
//! (self-map, contraction, analytic domain), a search limit, or exact endpoint
//! mantissas outgrowing that scalar limit; never exhausted work counters.
//! Integrations that stop before the requested horizon and rejected steps are
//! recorded as failed calls; set `ATLAS_DEBUG=1` to print each failure's status.
//! A watchdog cancels any single call that runs well past the atlas ceiling, so
//! one pathological input cannot stall a run; such points are recorded as
//! ceiling hits and end their series.
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    clock::ClockStatus,
    coordinates::{
        CoordinateField,
        qr_evolution::{QrEvolutionOptions, QrEvolutionStatus, QrProposalPolicy},
    },
    evolution::{EvolutionOptions, EvolutionRequest, EvolutionStatus},
    picard::{PicardProposal, PicardStatus},
    proposal::Rk4Options,
    reachability::{ReachabilityPolicy, ReachabilityRequest, ReachabilityStatus},
    sensitivity::SensitivityOptions,
    taylor::FieldTaylorRequest,
    taylor_flow::{
        FlowTaylorProgram, FlowTaylorStatus,
        adaptive::{AdaptiveTaylorOptions, AdaptiveTaylorStatus},
        evolution::FlowTaylorEvolutionStatus,
        noise::{
            FlowNoiseRequest, FlowNoiseState, FlowNoiseStatus,
            evolution::{FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus},
        },
    },
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
    work_budget::CancellationToken,
};
use perf_atlas::{Atlas, Rng, powers_of_two};
use std::{
    any::Any,
    cell::RefCell,
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

// ---------------------------------------------------------------------------
// Budgets and the watchdog
// ---------------------------------------------------------------------------

/// Per-stage scalar bit limit. Cartesian endpoints stay exact, so nonlinear
/// fields grow mantissas with every step; the default 16384 bits stops such
/// runs after a few steps. The atlas raises it to expose the cost growth; one
/// series keeps the default to show where that stop occurs.
static SCALAR_BITS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1 << 20);

/// Limits far above any swept case, so stops come from the mathematics.
fn policy() -> EnclosurePolicy {
    EnclosurePolicy {
        graph_entries: 100_000_000,
        graph_work: 1_000_000_000_000,
        scalar_bits: SCALAR_BITS.load(std::sync::atomic::Ordering::Relaxed),
        scalar_work: 1 << 62,
        ..EnclosurePolicy::default()
    }
}

/// The most recent watched budget's start time and cancellation token.
static WATCHED: Mutex<Option<(Instant, CancellationToken)>> = Mutex::new(None);

/// A fresh budget whose work the watchdog cancels after the call limit.
fn budget() -> EnclosureBudget {
    let token = CancellationToken::new();
    *WATCHED.lock().unwrap() = Some((Instant::now(), token.clone()));
    let mut budget = EnclosureBudget::new(policy());
    budget.set_cancellation(Some(token));
    budget
}

/// Cancel the current watched budget once it has run longer than `limit`.
/// Cancellation is cooperative, so a call returns at its next checked stage.
fn start_watchdog(limit: Duration) {
    std::thread::spawn(move || {
        loop {
            std::thread::sleep(Duration::from_millis(5));
            if let Some((start, token)) = WATCHED.lock().unwrap().as_ref()
                && start.elapsed() > limit
            {
                token.cancel();
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Exact inputs
// ---------------------------------------------------------------------------

/// State coordinates are exact dyadics on a 2^-48 grid.
const GRID: i32 = -48;

fn time(mantissa: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::point(mantissa.into(), exponent).unwrap()
}

/// `[center - 2^(w-1), center + 2^(w-1)]`, or the point `center` when `w` is None.
fn interval(center: f64, width_log2: Option<i32>) -> DyadicInterval {
    let c = (center * 2f64.powi(-GRID)).round() as i64;
    match width_log2 {
        None => DyadicInterval::point(c.into(), GRID).unwrap(),
        Some(w) => {
            let r = 1i64 << (w - 1 - GRID).clamp(0, 62);
            DyadicInterval::new((c - r).into(), (c + r).into(), GRID).unwrap()
        }
    }
}

fn rational(numerator: i64, denominator: i64) -> Expr {
    Expr::real(Real::new(
        Rational::from(numerator) / Rational::from(denominator),
    ))
}

/// `2^k` as an exact constant.
fn power_of_two(k: i32) -> Rational {
    if k >= 0 {
        Rational::from(1i64 << k)
    } else {
        Rational::from(1) / Rational::from(1i64 << -k)
    }
}

// ---------------------------------------------------------------------------
// Fields
// ---------------------------------------------------------------------------

/// A vector field with its default initial center and step caps.
#[derive(Clone)]
struct Spec {
    /// Cache key and series label.
    name: String,
    /// State dimension.
    n: usize,
    outputs: Vec<Expr>,
    center: Vec<f64>,
    /// log2 of the step cap for Taylor (order >= 1) integration.
    taylor_step: i32,
    /// log2 of the step cap for first-order Picard integration.
    picard_step: i32,
}

fn x(i: usize) -> Expr {
    Expr::anonymous_symbol(SymbolId(i as u32 + 1))
}

const T: SymbolId = SymbolId(0);

impl Spec {
    fn new(name: &str, outputs: Vec<Expr>, center: Vec<f64>, taylor: i32, picard: i32) -> Self {
        Self {
            name: name.to_owned(),
            n: outputs.len(),
            outputs,
            center,
            taylor_step: taylor,
            picard_step: picard,
        }
    }
    /// x' = -x.
    fn decay() -> Self {
        Self::new("linear decay", vec![-x(0)], vec![1.0], -3, -6)
    }
    /// x' = -y, y' = x.
    fn rotation() -> Self {
        Self::new("linear rotation", vec![-x(1), x(0)], vec![1.0, 0.0], -3, -6)
    }
    /// x' = x^2 from 1/2: blows up at t = 2.
    fn riccati() -> Self {
        Self::new("Riccati x'=x²", vec![x(0) * x(0)], vec![0.5], -5, -7)
    }
    /// Van der Pol with mu = 2^k.
    fn van_der_pol_mu(k: i32) -> Self {
        let mu = Expr::real(Real::new(power_of_two(k)));
        let outputs = vec![x(1), mu * (Expr::int(1) - x(0) * x(0)) * x(1) - x(0)];
        Self::new(
            &format!("Van der Pol mu=2^{k}"),
            outputs,
            vec![2.0, 0.0],
            -4,
            -7,
        )
    }
    fn van_der_pol() -> Self {
        Self {
            name: "Van der Pol".to_owned(),
            ..Self::van_der_pol_mu(0)
        }
    }
    /// x' = x - xy, y' = xy - y.
    fn lotka_volterra() -> Self {
        Self::new(
            "Lotka-Volterra",
            vec![x(0) - x(0) * x(1), x(0) * x(1) - x(1)],
            vec![1.5, 1.0],
            -4,
            -7,
        )
    }
    /// Classic Lorenz (10, 28, 8/3).
    fn lorenz() -> Self {
        Self::new(
            "Lorenz",
            vec![
                Expr::int(10) * (x(1) - x(0)),
                x(0) * (Expr::int(28) - x(2)) - x(1),
                x(0) * x(1) - rational(8, 3) * x(2),
            ],
            vec![1.0, 1.0, 1.0],
            -7,
            -10,
        )
    }
    /// Lorenz-96 with forcing 8 near its equilibrium, any dimension.
    fn lorenz96(n: usize) -> Self {
        let outputs = (0..n)
            .map(|i| {
                (x((i + 1) % n) - x((i + n - 2) % n)) * x((i + n - 1) % n) - x(i) + Expr::int(8)
            })
            .collect();
        let center = (0..n)
            .map(|i| if i == 0 { 8.015625 } else { 8.0 })
            .collect();
        Self::new(&format!("Lorenz-96 n={n}"), outputs, center, -6, -9)
    }
    /// Bidiagonal linear chain x_i' = -x_i + x_{i+1}.
    fn chain(n: usize) -> Self {
        let outputs = (0..n)
            .map(|i| if i + 1 < n { x(i + 1) - x(i) } else { -x(i) })
            .collect();
        Self::new(
            &format!("linear chain n={n}"),
            outputs,
            vec![1.0; n],
            -3,
            -6,
        )
    }
    /// Dense linear x_i' = -x_i + sum_j (j-i)/(4n) x_j.
    fn dense(n: usize) -> Self {
        let outputs = (0..n)
            .map(|i| {
                (0..n).fold(-x(i), |sum, j| {
                    if i == j {
                        sum
                    } else {
                        sum + rational(j as i64 - i as i64, 4 * n as i64) * x(j)
                    }
                })
            })
            .collect();
        Self::new(
            &format!("dense linear n={n}"),
            outputs,
            vec![1.0; n],
            -3,
            -6,
        )
    }
    /// x' = -x^p.
    fn power(p: i64) -> Self {
        Self::new(
            &format!("x'=-x^{p}"),
            vec![-x(0).powi(p)],
            vec![1.0],
            -4,
            -7,
        )
    }
    /// x' = -sum_{k=1..p} x^k / k, written as separate monomials.
    fn polynomial(p: i64) -> Self {
        let rhs = (1..=p).fold(Expr::int(0), |sum, k| sum - rational(1, k) * x(0).powi(k));
        Self::new(&format!("poly deg {p}"), vec![rhs], vec![0.5], -4, -7)
    }
    /// x' = -(x*x*...*x), a product chain of p factors.
    fn product(p: i64) -> Self {
        let rhs = (1..p).fold(x(0), |prod, _| prod * x(0));
        Self::new(&format!("product {p}"), vec![-rhs], vec![1.0], -4, -7)
    }
    /// The same field with a randomly moved center.
    fn jitter(&self, rng: &mut Rng, scale: f64) -> Self {
        let mut out = self.clone();
        for c in &mut out.center {
            *c += (rng.unit() - 0.5) * scale;
        }
        out
    }
    fn definition(&self) -> FieldDefinition {
        let state: Vec<_> = (0..self.n).map(|i| SymbolId(i as u32 + 1)).collect();
        let bindings: Vec<_> = std::iter::once(T).chain(state.iter().copied()).collect();
        let source = ExpressionProgram::compile(
            &self.outputs,
            &bindings,
            &mut ExpressionBudget::new(100_000_000, 1_000_000_000),
        )
        .unwrap();
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state,
        }
    }
}

/// Steps for fixed short horizons: Lorenz's exact Cartesian endpoints grow
/// fastest, so it gets half the horizon.
fn short(spec: &Spec) -> u64 {
    if spec.name == "Lorenz" { 2 } else { 4 }
}

thread_local! {
    static CACHE: RefCell<HashMap<String, Box<dyn Any>>> = RefCell::new(HashMap::new());
}

/// Build once per key; prepared fields and programs share storage when cloned.
fn cached<V: Clone + 'static>(key: String, build: impl FnOnce() -> V) -> V {
    if let Some(v) = CACHE.with(|c| {
        c.borrow()
            .get(&key)
            .map(|v| v.downcast_ref::<V>().unwrap().clone())
    }) {
        return v;
    }
    let v = build();
    CACHE.with(|c| c.borrow_mut().insert(key, Box::new(v.clone())));
    v
}

fn field<const N: usize>(spec: &Spec) -> ExpressionField<N> {
    cached(format!("field:{}", spec.name), || {
        ExpressionField::from_definition(spec.definition(), &mut EnclosureBudget::new(policy()))
            .unwrap()
    })
}

fn program<const N: usize>(spec: &Spec, order: usize) -> FlowTaylorProgram<N> {
    cached(format!("program:{}:{order}", spec.name), || {
        field::<N>(spec)
            .flow_taylor(order, &mut EnclosureBudget::new(policy()))
            .unwrap()
    })
}

fn initial<const N: usize>(spec: &Spec, width: Option<i32>) -> BoxN<DyadicInterval, N> {
    BoxN::new(std::array::from_fn(|i| interval(spec.center[i], width))).unwrap()
}

/// Horizon `[0, steps * 2^step]`.
fn request<const N: usize>(
    spec: &Spec,
    width: Option<i32>,
    steps: u64,
    step: i32,
) -> EvolutionRequest<N> {
    EvolutionRequest {
        start: time(0, 0),
        end: time(steps as i64, step),
        initial: initial(spec, width),
    }
}

/// Step cap `2^step`, with room for halvings over `steps` nominal steps.
fn options(steps: u64, step: i32) -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(time(1, step)),
        maximum_steps: (steps as usize).saturating_mul(64).saturating_add(256),
        ..EvolutionOptions::default()
    }
}

/// A Picard proposal for one step of `2^step`, found by the native search
/// without halving. A rejected search yields its last (rejected) candidate.
fn proposal<const N: usize>(
    f: &ExpressionField<N>,
    initial: BoxN<DyadicInterval, N>,
    step: i32,
) -> PicardProposal<N> {
    let search = |expansions| {
        f.integrate(
            EvolutionRequest {
                start: time(0, 0),
                end: time(1, step),
                initial: initial.clone(),
            },
            EvolutionOptions {
                maximum_step: Some(time(1, step)),
                maximum_steps: 1,
                maximum_halvings: 0,
                maximum_expansions: expansions,
            },
            &mut budget(),
        )
    };
    // Few expansions first: a failing step's candidates otherwise grow without
    // bound. Narrow boxes may need many doublings to reach the trajectory tube.
    let first = search(12);
    if let Some(segment) = first.segments().next() {
        return segment.proposal().clone();
    }
    if let Some(segment) = search(48).segments().next() {
        return segment.proposal().clone();
    }
    first
        .attempts()
        .last()
        .map(|a| a.report().proposal().clone())
        .unwrap_or(PicardProposal {
            start: time(0, 0),
            end: time(1, step),
            initial: initial.clone(),
            candidate: initial,
        })
}

fn identity_chart<const N: usize>(spec: &Spec) -> CoordinateField<N> {
    cached(format!("chart:{}", spec.name), || {
        let a: Vec<Vec<Real>> = (0..N)
            .map(|i| {
                (0..N)
                    .map(|j| if i == j { Real::one() } else { Real::zero() })
                    .collect()
            })
            .collect();
        field::<N>(spec)
            .in_coordinates(
                CoordinateChange {
                    variables: (0..N).map(|i| SymbolId(i as u32 + 1)).collect(),
                    to_source: a.clone(),
                    from_source: a,
                    origin: vec![Real::zero(); N],
                },
                &mut EnclosureBudget::new(policy()),
            )
            .unwrap()
    })
}

/// Unit upper-bidiagonal `A = I + superdiagonal`, its exact inverse and the
/// spec's center as origin.
fn bidiagonal_change(spec: &Spec) -> CoordinateChange {
    let n = spec.n;
    let to_source = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| Real::from(i64::from(i == j || j == i + 1)))
                .collect()
        })
        .collect();
    let from_source = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    if j < i {
                        Real::zero()
                    } else if (j - i) % 2 == 0 {
                        Real::one()
                    } else {
                        -Real::one()
                    }
                })
                .collect()
        })
        .collect();
    CoordinateChange {
        variables: (0..n).map(|i| SymbolId(i as u32 + 1)).collect(),
        to_source,
        from_source,
        origin: spec
            .center
            .iter()
            .map(|&c| Real::new(Rational::from((c * 1024.0).round() as i64) / Rational::from(1024)))
            .collect(),
    }
}

// ---------------------------------------------------------------------------
// Timed operations
// ---------------------------------------------------------------------------

/// Set `ATLAS_DEBUG=1` to print the status of every failed timed call.
fn debug() -> bool {
    static DEBUG: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *DEBUG.get_or_init(|| std::env::var_os("ATLAS_DEBUG").is_some())
}

/// Whether a report status matches its success pattern; stopped and rejected
/// outcomes are failures.
macro_rules! done {
    ($status:expr, $pattern:pat) => {
        match $status {
            status => {
                let ok = matches!(status, $pattern);
                if !ok && debug() {
                    eprintln!("    failed: {status:?}");
                }
                ok
            }
        }
    };
}

/// The timed call: everything it needs is prepared outside the timer.
type Job = Box<dyn Fn() -> bool>;

/// Run a job under the default per-stage scalar bit limit.
fn default_bits(job: Job) -> Job {
    Box::new(move || {
        let atlas_bits = SCALAR_BITS.swap(
            EnclosurePolicy::default().scalar_bits,
            std::sync::atomic::Ordering::Relaxed,
        );
        let ok = job();
        SCALAR_BITS.store(atlas_bits, std::sync::atomic::Ordering::Relaxed);
        ok
    })
}

fn call(job: &Job) -> bool {
    job()
}

const EVERY: QrProposalPolicy = QrProposalPolicy::EveryWindow;
const BACKOFF: QrProposalPolicy = QrProposalPolicy::Backoff {
    maximum_interval: 16,
};

/// Whole-horizon integration method.
#[derive(Clone, Copy)]
enum Method {
    /// Native first-order Picard steps (`ExpressionField::integrate`).
    Picard,
    /// Cartesian Taylor evolution (`FlowTaylorProgram::integrate`).
    Taylor(usize),
    /// Correlated affine-noise evolution (`integrate_noise`) with a column cap.
    Noise(usize, usize),
    /// QR-frame evolution (`integrate_qr`) with a Taylor order and proposal schedule.
    Qr(usize, QrProposalPolicy),
    /// Adaptive Taylor order up to the maximum, with an endpoint remainder limit 2^k.
    Adaptive(usize, i32),
}

#[derive(Clone)]
enum Op {
    FromDefinition,
    Image(Option<i32>),
    Jacobian(Option<i32>),
    TaylorModel {
        order: usize,
        width: Option<i32>,
    },
    Prepare(usize),
    Restrict {
        from: usize,
        to: usize,
    },
    Picard {
        step: i32,
        width: Option<i32>,
    },
    TaylorStep {
        order: usize,
        step: i32,
        width: Option<i32>,
    },
    NoiseStep {
        order: usize,
        step: i32,
        width: Option<i32>,
        noise: usize,
    },
    Rk4 {
        step: i32,
        width: Option<i32>,
        certify: bool,
    },
    Integrate {
        method: Method,
        steps: u64,
        step: Option<i32>,
        width: Option<i32>,
    },
    Reach {
        steps: u64,
        width: Option<i32>,
        epsilon: i32,
    },
    RationalTime {
        steps: u64,
    },
    Sensitivity {
        method: Method,
        steps: u64,
        terms: usize,
        width: i32,
    },
    InCoordinates,
    InQr,
    Transition {
        enclose: bool,
    },
}

fn job(spec: &Spec, op: Op) -> Job {
    macro_rules! with_n {
        ($($n:literal)*) => {
            match spec.n {
                $($n => { job_n::<$n>(spec, op) })*
                n => panic!("unsupported dimension {n}"),
            }
        };
    }
    with_n!(1 2 3 4 6 8)
}

fn job_n<const N: usize>(spec: &Spec, op: Op) -> Job {
    match op {
        Op::FromDefinition => {
            let definition = spec.definition();
            Box::new(move || {
                ExpressionField::<N>::from_definition(definition.clone(), &mut budget()).is_ok()
            })
        }
        Op::Image(width) => {
            let (f, b) = (field::<N>(spec), initial::<N>(spec, width));
            Box::new(move || f.image(&time(0, 0), &b, &mut budget()).is_ok())
        }
        Op::Jacobian(width) => {
            let (f, b) = (field::<N>(spec), initial::<N>(spec, width));
            Box::new(move || f.jacobian(&time(0, 0), &b, &mut budget()).is_ok())
        }
        Op::TaylorModel { order, width } => {
            let f = field::<N>(spec);
            let request = FieldTaylorRequest {
                time: DyadicInterval::new(0.into(), 1.into(), spec.taylor_step).unwrap(),
                state: initial::<N>(spec, width),
                order,
            };
            Box::new(move || f.taylor_model(request.clone(), &mut budget()).is_ok())
        }
        Op::Prepare(order) => {
            let f = field::<N>(spec);
            Box::new(move || f.flow_taylor(order, &mut budget()).is_ok())
        }
        Op::Restrict { from, to } => {
            let p = program::<N>(spec, from);
            Box::new(move || p.restrict_order(to, &mut budget()).is_ok())
        }
        Op::Picard { step, width } => {
            let f = field::<N>(spec);
            let p = proposal(&f, initial(spec, width), step);
            Box::new(move || {
                done!(
                    f.certify_step(p.clone(), &mut budget()).status(),
                    PicardStatus::Certified
                )
            })
        }
        Op::TaylorStep { order, step, width } => {
            let program = program::<N>(spec, order);
            let p = proposal(&field::<N>(spec), initial(spec, width), step);
            Box::new(move || {
                done!(
                    program.certify_step(p.clone(), &mut budget()).status(),
                    FlowTaylorStatus::Certified
                )
            })
        }
        Op::NoiseStep {
            order,
            step,
            width,
            noise,
        } => {
            let program = program::<N>(spec, order);
            let start = initial::<N>(spec, width);
            let p = proposal(&field::<N>(spec), start.clone(), step);
            let state = Arc::new(
                FlowNoiseState::new(time(0, 0), start, &mut EnclosureBudget::new(policy()))
                    .unwrap(),
            );
            let request = FlowNoiseRequest {
                end: time(1, step),
                candidate: p.candidate,
                maximum_noise: noise,
                round_coefficients: true,
                column_cutoff_bits: Some(16),
            };
            Box::new(move || {
                done!(
                    program
                        .certify_noise_step(state.clone(), request.clone(), &mut budget())
                        .status(),
                    FlowNoiseStatus::Certified
                )
            })
        }
        Op::Rk4 {
            step,
            width,
            certify,
        } => {
            let f = field::<N>(spec);
            let proposer = f.rk4_proposer(&mut EnclosureBudget::new(policy())).unwrap();
            let request = request::<N>(spec, width, 1, step);
            Box::new(move || {
                let mut b = budget();
                match proposer.propose(request.clone(), Rk4Options::default(), &mut b) {
                    Ok(p) if certify => {
                        done!(p.certify(&mut b).proof().status(), PicardStatus::Certified)
                    }
                    Ok(_) => true,
                    Err(_) => false,
                }
            })
        }
        Op::Integrate {
            method,
            steps,
            step,
            width,
        } => integrate::<N>(spec, method, steps, step, width),
        Op::Sensitivity {
            method,
            steps,
            terms,
            width,
        } => sensitivity::<N>(spec, method, steps, terms, width),
        Op::Reach {
            steps,
            width,
            epsilon,
        } => {
            let f = field::<N>(spec);
            let step = spec.picard_step;
            let request = ReachabilityRequest {
                evolution: request::<N>(spec, width, steps, step),
                epsilon: time(1, epsilon),
            };
            let policy = ReachabilityPolicy {
                evolution: options(steps, step),
                ..ReachabilityPolicy::default()
            };
            Box::new(move || {
                done!(
                    f.reach(request.clone(), policy.clone(), &mut budget())
                        .status(),
                    ReachabilityStatus::Completed
                )
            })
        }
        Op::RationalTime { steps } => {
            // Physical span [0, steps/3]; tau steps keep physical steps <= 2^picard_step.
            let f = field::<N>(spec);
            let clock = f
                .on_time_span(
                    Rational::from(0),
                    Rational::from(steps as i64) / Rational::from(3),
                    &mut EnclosureBudget::new(policy()),
                )
                .unwrap();
            let tau = -(((steps as f64 / 3.0).log2().ceil() as i32) - spec.picard_step);
            let start = initial::<N>(spec, Some(-20));
            let opts = options(steps, tau);
            Box::new(move || {
                done!(
                    clock
                        .integrate(start.clone(), opts.clone(), &mut budget())
                        .status(),
                    ClockStatus::Completed
                )
            })
        }
        Op::InCoordinates => {
            let f = field::<N>(spec);
            let change = bidiagonal_change(spec);
            Box::new(move || f.in_coordinates(change.clone(), &mut budget()).is_ok())
        }
        Op::InQr => {
            // Lower-triangular all-ones basis: orthonormalization needs square roots.
            let f = field::<N>(spec);
            let basis: Vec<Vec<Real>> = (0..N)
                .map(|i| (0..N).map(|j| Real::from(i64::from(j <= i))).collect())
                .collect();
            let origin = bidiagonal_change(spec).origin;
            Box::new(move || {
                f.in_qr_coordinates(basis.clone(), origin.clone(), &mut budget())
                    .is_ok()
            })
        }
        Op::Transition { enclose } => {
            let from = identity_chart::<N>(spec);
            let to = field::<N>(spec)
                .in_coordinates(bidiagonal_change(spec), &mut EnclosureBudget::new(policy()))
                .unwrap();
            if enclose {
                let transition = from
                    .transition_to(&to, &mut EnclosureBudget::new(policy()))
                    .unwrap();
                let b = initial::<N>(spec, Some(-8));
                Box::new(move || transition.enclose(b.clone(), &mut budget()).is_ok())
            } else {
                Box::new(move || from.transition_to(&to, &mut budget()).is_ok())
            }
        }
    }
}

fn integrate<const N: usize>(
    spec: &Spec,
    method: Method,
    steps: u64,
    step: Option<i32>,
    width: Option<i32>,
) -> Job {
    let step = step.unwrap_or(match method {
        Method::Picard => spec.picard_step,
        Method::Taylor(0) | Method::Noise(0, _) | Method::Qr(0, _) => spec.picard_step,
        _ => spec.taylor_step,
    });
    let request = request::<N>(spec, width, steps, step);
    let opts = options(steps, step);
    match method {
        Method::Picard => {
            let f = field::<N>(spec);
            Box::new(move || {
                done!(
                    f.integrate(request.clone(), opts.clone(), &mut budget())
                        .status(),
                    EvolutionStatus::Completed
                )
            })
        }
        Method::Taylor(order) => {
            let p = program::<N>(spec, order);
            Box::new(move || {
                done!(
                    p.integrate(request.clone(), opts.clone(), &mut budget())
                        .status(),
                    FlowTaylorEvolutionStatus::Completed
                )
            })
        }
        Method::Noise(order, noise) => {
            let p = program::<N>(spec, order);
            let state = Arc::new(
                FlowNoiseState::new(
                    time(0, 0),
                    request.initial,
                    &mut EnclosureBudget::new(policy()),
                )
                .unwrap(),
            );
            let options = FlowNoiseEvolutionOptions {
                maximum_noise: noise,
                native: opts,
                ..FlowNoiseEvolutionOptions::default()
            };
            let end = request.end;
            Box::new(move || {
                done!(
                    p.integrate_noise(state.clone(), end.clone(), options.clone(), &mut budget())
                        .status(),
                    FlowNoiseEvolutionStatus::Completed
                )
            })
        }
        Method::Qr(order, proposals) => {
            let chart = identity_chart::<N>(spec);
            // One frame selection per four nominal steps.
            let windows = steps.div_ceil(4);
            let options = QrEvolutionOptions {
                window: time(1, step + 2),
                maximum_windows: windows as usize + 4,
                taylor_order: (order > 0).then_some(order),
                native: opts,
                proposals,
            };
            Box::new(move || {
                done!(
                    chart
                        .integrate_qr(request.clone(), options.clone(), &mut budget())
                        .status(),
                    QrEvolutionStatus::Completed
                )
            })
        }
        Method::Adaptive(maximum_order, limit) => {
            let controller = field::<N>(spec).adaptive_taylor(AdaptiveTaylorOptions {
                minimum_order: 1,
                maximum_order,
                remainder_limits: std::array::from_fn(|_| time(1, limit)),
                maximum_quality_halvings: 16,
                native: opts,
            });
            Box::new(move || {
                done!(
                    controller
                        .integrate(request.clone(), &mut budget())
                        .status(),
                    AdaptiveTaylorStatus::Completed
                )
            })
        }
    }
}

/// Times `sensitivity` over a source evolution built once, outside the timer,
/// from a box of width 2^width. Stopped sources are marked as failures.
fn sensitivity<const N: usize>(
    spec: &Spec,
    method: Method,
    steps: u64,
    terms: usize,
    width: i32,
) -> Job {
    let step = match method {
        Method::Picard | Method::Taylor(0) | Method::Noise(0, _) | Method::Qr(0, _) => {
            spec.picard_step
        }
        _ => spec.taylor_step,
    };
    let request = request::<N>(spec, Some(width), steps, step);
    let opts = options(steps, step);
    let options = SensitivityOptions { terms };
    let build = || EnclosureBudget::new(policy());
    macro_rules! timed {
        ($source:expr, $completed:pat) => {{
            let source = Arc::new($source);
            Box::new(move || match source.sensitivity(options, &mut budget()) {
                Ok(s) => done!(s.source().status(), $completed),
                Err(error) => {
                    if debug() {
                        eprintln!("    failed: {error:?}");
                    }
                    false
                }
            })
        }};
    }
    match method {
        Method::Picard => timed!(
            field::<N>(spec).integrate(request, opts, &mut build()),
            EvolutionStatus::Completed
        ),
        Method::Taylor(order) => timed!(
            program::<N>(spec, order).integrate(request, opts, &mut build()),
            FlowTaylorEvolutionStatus::Completed
        ),
        Method::Qr(order, proposals) => timed!(
            identity_chart::<N>(spec).integrate_qr(
                request,
                QrEvolutionOptions {
                    window: time(1, step + 2),
                    maximum_windows: steps.div_ceil(4) as usize + 4,
                    taylor_order: (order > 0).then_some(order),
                    native: opts,
                    proposals,
                },
                &mut build()
            ),
            QrEvolutionStatus::Completed
        ),
        Method::Adaptive(maximum_order, limit) => timed!(
            field::<N>(spec)
                .adaptive_taylor(AdaptiveTaylorOptions {
                    minimum_order: 1,
                    maximum_order,
                    remainder_limits: std::array::from_fn(|_| time(1, limit)),
                    maximum_quality_halvings: 16,
                    native: opts,
                })
                .integrate(request, &mut build()),
            AdaptiveTaylorStatus::Completed
        ),
        Method::Noise(order, noise) => timed!(
            program::<N>(spec, order).integrate_noise(
                Arc::new(FlowNoiseState::new(time(0, 0), request.initial, &mut build()).unwrap()),
                request.end,
                FlowNoiseEvolutionOptions {
                    maximum_noise: noise,
                    native: opts,
                    ..FlowNoiseEvolutionOptions::default()
                },
                &mut build()
            ),
            FlowNoiseEvolutionStatus::Completed
        ),
    }
}

// ---------------------------------------------------------------------------
// Sweeps
// ---------------------------------------------------------------------------

fn ints(values: &[i32]) -> Vec<f64> {
    values.iter().map(|&v| f64::from(v)).collect()
}

/// Random log2 width in `lo..=hi`.
fn random_width(rng: &mut Rng, lo: i32, hi: i32) -> Option<i32> {
    Some(rng.range(i64::from(lo), i64::from(hi)) as i32)
}

fn main() {
    let mut atlas = Atlas::from_args("hyperdynamics");
    let quick = atlas.config().quick;
    start_watchdog(atlas.config().single_ceiling * 3 / 2);

    let decay = Spec::decay();
    let rotation = Spec::rotation();
    let riccati = Spec::riccati();
    let vdp = Spec::van_der_pol();
    let lv = Spec::lotka_volterra();
    let lorenz = Spec::lorenz();
    let nonlinear = [&riccati, &vdp, &lv, &lorenz];
    let evolving = [&rotation, &riccati, &vdp, &lorenz];

    let dims: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 6.0, 8.0];
    let dim = |m: f64| m as usize;
    let orders: Vec<f64> = ints(&[0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16]);
    let widths: Vec<f64> = ints(&[-40, -32, -24, -16, -12, -8, -6, -4, -3, -2, -1, 0, 1, 2]);
    let width = |m: f64| Some(m as i32);
    let horizons = powers_of_two(0, if quick { 8 } else { 11 });
    let steps = |m: f64| m as u64;
    let noise_width = Some(-6);
    let near_point = Some(-20);

    // ----- field construction and images -----
    atlas.operation(
        "field",
        "from_definition by dimension",
        "state dimension",
        "ExpressionField::from_definition: validate the source and regenerate the symbolic Jacobian, for N-dimensional families.",
    );
    for (kind, family) in [
        ("linear chain", Spec::chain as fn(usize) -> Spec),
        ("dense linear", Spec::dense),
        ("Lorenz-96", Spec::lorenz96),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| job(&family(dim(m)), Op::FromDefinition),
            call,
        );
    }

    atlas.operation(
        "field",
        "from_definition by nonlinearity",
        "polynomial degree",
        "ExpressionField::from_definition for a one-dimensional field of growing polynomial degree (Jacobian regeneration grows with the expression).",
    );
    let degrees = powers_of_two(0, 6);
    for (kind, family) in [
        ("monomial -x^p", Spec::power as fn(i64) -> Spec),
        ("dense polynomial", Spec::polynomial),
        ("product chain", Spec::product),
    ] {
        atlas.sweep(
            kind,
            &degrees,
            |m, _| job(&family(m as i64), Op::FromDefinition),
            call,
        );
    }

    atlas.operation(
        "field",
        "image by initial width",
        "log2 state-box width",
        "EnclosedVectorField::image: interval enclosure of f(t, X) on a state box of width 2^k around the field's reference point.",
    );
    for spec in nonlinear {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| job(spec, Op::Image(width(m))),
            call,
        );
    }
    atlas.fuzz(
        "Van der Pol, random center and width",
        &ints(&[-32, -16, -8, -4, -2, 0]),
        |m, rng| {
            job(
                &vdp.jitter(rng, 8.0),
                Op::Image(random_width(rng, m as i32 - 4, m as i32)),
            )
        },
        call,
    );

    atlas.operation(
        "field",
        "jacobian by initial width",
        "log2 state-box width",
        "EnclosedVectorField::jacobian: interval enclosure of the state Jacobian on a box of width 2^k.",
    );
    for spec in nonlinear {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| job(spec, Op::Jacobian(width(m))),
            call,
        );
    }

    atlas.operation(
        "field",
        "taylor_model by order",
        "total polynomial degree",
        "ExpressionField::taylor_model: certified multivariate Taylor model of the field on a time/state box (width 2^-4).",
    );
    for spec in nonlinear {
        atlas.sweep(
            &spec.name,
            &ints(&[0, 1, 2, 3, 4, 5, 6, 8, 10]),
            |m, _| {
                job(
                    spec,
                    Op::TaylorModel {
                        order: m as usize,
                        width: Some(-4),
                    },
                )
            },
            call,
        );
    }

    // ----- Taylor flow preparation -----
    atlas.operation(
        "taylor-flow",
        "flow_taylor preparation by order",
        "Taylor order",
        "ExpressionField::flow_taylor: build the Lie-derivative coefficient programs through the next-order remainder.",
    );
    for spec in [&decay, &riccati, &vdp, &lorenz] {
        atlas.sweep(
            &spec.name,
            &orders,
            |m, _| job(spec, Op::Prepare(m as usize)),
            call,
        );
    }

    atlas.operation(
        "taylor-flow",
        "flow_taylor preparation by dimension",
        "state dimension",
        "ExpressionField::flow_taylor at order 3 for N-dimensional families.",
    );
    for (kind, family) in [
        ("linear chain", Spec::chain as fn(usize) -> Spec),
        ("dense linear", Spec::dense),
        ("Lorenz-96", Spec::lorenz96),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| job(&family(dim(m)), Op::Prepare(3)),
            call,
        );
    }

    atlas.operation(
        "taylor-flow",
        "flow_taylor preparation by nonlinearity",
        "polynomial degree p of x' = -x^p",
        "ExpressionField::flow_taylor for x' = -x^p at several orders: coefficient programs grow with both degree and order.",
    );
    for order in [2usize, 4, 8] {
        atlas.sweep(
            &format!("order {order}"),
            &powers_of_two(0, 6),
            |m, _| job(&Spec::power(m as i64), Op::Prepare(order)),
            call,
        );
    }

    atlas.operation(
        "taylor-flow",
        "restrict_order from order 10",
        "restricted order",
        "FlowTaylorProgram::restrict_order: share a prepared order-10 program's stages at a lower order.",
    );
    for spec in nonlinear {
        atlas.sweep(
            &spec.name,
            &ints(&[0, 1, 2, 3, 4, 6, 8, 10]),
            |m, _| {
                job(
                    spec,
                    Op::Restrict {
                        from: 10,
                        to: m as usize,
                    },
                )
            },
            call,
        );
    }

    // ----- single steps -----
    let step_sizes = ints(&[-20, -16, -12, -10, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1]);
    atlas.operation(
        "step",
        "Picard certify_step by step size",
        "log2 step size",
        "ExpressionField::certify_step on a proposal found by the native search (no halving), box width 2^-10. Large steps fail the self-map or contraction checks.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &step_sizes,
            |m, _| {
                job(
                    spec,
                    Op::Picard {
                        step: m as i32,
                        width: Some(-10),
                    },
                )
            },
            call,
        );
    }
    atlas.fuzz(
        "Lotka-Volterra, random center and width",
        &ints(&[-12, -8, -6, -4, -2]),
        |m, rng| {
            job(
                &lv.jitter(rng, 1.0),
                Op::Picard {
                    step: m as i32,
                    width: random_width(rng, -24, -2),
                },
            )
        },
        call,
    );

    atlas.operation(
        "step",
        "Taylor certify_step by order",
        "Taylor order",
        "FlowTaylorProgram::certify_step: Picard existence, analytic-domain proof and the trajectory Taylor formula for one step of each field's Taylor step cap, box width 2^-10.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &orders,
            |m, _| {
                job(
                    spec,
                    Op::TaylorStep {
                        order: m as usize,
                        step: spec.taylor_step,
                        width: Some(-10),
                    },
                )
            },
            call,
        );
    }
    atlas.grid(
        "Van der Pol",
        ("Taylor order", &ints(&[0, 1, 2, 3, 4, 6, 8])),
        ("log2 step size", &ints(&[-10, -8, -6, -4, -3, -2, -1])),
        |order, step, _| {
            job(
                &vdp,
                Op::TaylorStep {
                    order: order as usize,
                    step: step as i32,
                    width: Some(-10),
                },
            )
        },
        call,
    );

    atlas.operation(
        "step",
        "Taylor certify_step by initial width",
        "log2 state-box width",
        "FlowTaylorProgram::certify_step at order 4 over one Taylor step cap, for initial boxes of width 2^k.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| {
                job(
                    spec,
                    Op::TaylorStep {
                        order: 4,
                        step: spec.taylor_step,
                        width: width(m),
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "step",
        "affine-noise certify_noise_step by initial width",
        "log2 state-box width",
        "FlowTaylorProgram::certify_noise_step at order 4: one correlated affine endpoint step (16 retained columns) for boxes of width 2^k.",
    );
    for spec in [&rotation, &vdp, &lv, &lorenz] {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| {
                job(
                    spec,
                    Op::NoiseStep {
                        order: 4,
                        step: spec.taylor_step,
                        width: width(m),
                        noise: 16,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "step",
        "RK4 proposal by step size",
        "log2 step size",
        "Rk4Proposer::propose: floating-point RK4 stages and a padded candidate box (no proof), box width 2^-10.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &step_sizes,
            |m, _| {
                job(
                    spec,
                    Op::Rk4 {
                        step: m as i32,
                        width: Some(-10),
                        certify: false,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "step",
        "RK4 proposal + certify by step size",
        "log2 step size",
        "Rk4Proposer::propose followed by exact Picard certification of the padded RK4 candidate; failures are candidates the verifier rejects.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &step_sizes,
            |m, _| {
                job(
                    spec,
                    Op::Rk4 {
                        step: m as i32,
                        width: Some(-10),
                        certify: true,
                    },
                )
            },
            call,
        );
    }

    // ----- whole-horizon evolution -----
    atlas.operation(
        "evolution",
        "Picard integrate by horizon",
        "horizon (multiples of the Picard step cap)",
        "ExpressionField::integrate (first-order native Picard steps) over n step caps, near-point box (width 2^-20). Exact Cartesian endpoints of nonlinear fields grow by several bits per step until the per-stage scalar bit limit (2^20 here) stops the run.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Picard,
                        steps: steps(m),
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "initial-state sensitivity by horizon",
        "horizon (multiples of the Picard step cap)",
        "EvolutionReport::sensitivity with 8 series terms, over a completed first-order Picard evolution of n step caps from a near-point box (width 2^-20). The evolution is built outside the timer; each call derives every segment's Peano–Baker terms, tail and composed derivative. Stopped sources are marked as failures.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                job(
                    spec,
                    Op::Sensitivity {
                        method: Method::Picard,
                        steps: steps(m),
                        terms: 8,
                        width: -20,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "initial-state sensitivity by series terms",
        "Peano–Baker terms per segment",
        "EvolutionReport::sensitivity over 16 Picard step caps, by the number of nonconstant series terms per segment. Zero terms keep identity plus the certified tail.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &orders,
            |m, _| {
                job(
                    spec,
                    Op::Sensitivity {
                        method: Method::Picard,
                        steps: 16,
                        terms: m as usize,
                        width: -20,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "initial-state sensitivity by source method",
        "horizon (multiples of the source step cap)",
        "sensitivity with 8 series terms over completed Picard, order-4 Taylor, adaptive Taylor (orders 1..=6, endpoint remainder limit 2^-20), order-3 QR-frame and order-4 affine-noise (2 columns, default rounding and cutoff; widths matched 64 columns on Van der Pol at a tenth of the build time) evolutions of n step caps from a box of width 2^-6, as in the QR integrate sweep (near-point boxes exhaust scalar size in the first QR window). Each source is built outside the timer; every method derives from the Picard proofs its segments retain, and QR adds two chart products per segment plus one exact transition per frame change.",
    );
    for (label, method) in [
        ("Picard", Method::Picard),
        ("Taylor 4", Method::Taylor(4)),
        ("adaptive Taylor", Method::Adaptive(6, -20)),
        ("QR frames", Method::Qr(3, EVERY)),
        ("affine noise", Method::Noise(4, 2)),
    ] {
        for spec in [&rotation, &vdp] {
            atlas.sweep(
                &format!("{}, {label}", spec.name),
                &horizons,
                |m, _| {
                    job(
                        spec,
                        Op::Sensitivity {
                            method,
                            steps: steps(m),
                            terms: 8,
                            width: -6,
                        },
                    )
                },
                call,
            );
        }
    }

    atlas.operation(
        "evolution",
        "Taylor integrate by horizon",
        "horizon (multiples of the Taylor step cap)",
        "FlowTaylorProgram::integrate at order 4 (Cartesian boxes) over n step caps, near-point box. Cartesian endpoints stay exact, so nonlinear fields grow endpoint mantissas every step until the per-stage scalar bit limit stops them (atlas: 2^20 bits; last series: the default 16384). Riccati also blows up at t = 2 (n = 64).",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Taylor(4),
                        steps: steps(m),
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }
    atlas.sweep(
        "Van der Pol, default 16384-bit policy",
        &horizons,
        |m, _| {
            default_bits(job(
                &vdp,
                Op::Integrate {
                    method: Method::Taylor(4),
                    steps: steps(m),
                    step: None,
                    width: near_point,
                },
            ))
        },
        call,
    );
    atlas.fuzz(
        "Van der Pol, random center",
        &powers_of_two(0, 6),
        |m, rng| {
            job(
                &vdp.jitter(rng, 2.0),
                Op::Integrate {
                    method: Method::Taylor(4),
                    steps: steps(m),
                    step: None,
                    width: near_point,
                },
            )
        },
        call,
    );
    atlas.grid(
        "Van der Pol",
        ("Taylor order", &ints(&[1, 2, 3, 4, 6, 8])),
        ("horizon (step caps)", &powers_of_two(0, 8)),
        |order, n, _| {
            job(
                &vdp,
                Op::Integrate {
                    method: Method::Taylor(order as usize),
                    steps: steps(n),
                    step: None,
                    width: near_point,
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "Taylor integrate by order",
        "Taylor order",
        "FlowTaylorProgram::integrate over 4 step caps (Lorenz: 2), near-point box, by order (order 0 uses the Picard step cap).",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &orders,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Taylor(m as usize),
                        steps: short(spec),
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "Taylor integrate by step count",
        "steps n over a fixed horizon (step cap = horizon / n)",
        "FlowTaylorProgram::integrate at order 4 over a fixed horizon of two Taylor step caps with step cap horizon/n. Few large steps risk halvings and wide tubes; many small steps multiply work and, for nonlinear fields, endpoint mantissa growth.",
    );
    for spec in evolving {
        let horizon_log2 = spec.taylor_step + 1;
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                let n = steps(m);
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Taylor(4),
                        steps: n,
                        step: Some(horizon_log2 - n.ilog2() as i32),
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "Taylor integrate by initial width",
        "log2 state-box width",
        "FlowTaylorProgram::integrate at order 4 over 4 step caps (Lorenz: 2) for boxes of width 2^k: wide boxes widen the tubes and make steps fail.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Taylor(4),
                        steps: short(spec),
                        step: None,
                        width: width(m),
                    },
                )
            },
            call,
        );
    }
    let grid_widths = ints(&[-24, -16, -10, -6, -4, -2, 0]);
    let grid_horizons = powers_of_two(0, 7);
    atlas.grid(
        "linear rotation, Cartesian",
        ("log2 state-box width", &grid_widths),
        ("horizon (step caps)", &grid_horizons),
        |w, n, _| {
            job(
                &rotation,
                Op::Integrate {
                    method: Method::Taylor(4),
                    steps: steps(n),
                    step: None,
                    width: Some(w as i32),
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "Taylor integrate by dimension",
        "state dimension",
        "FlowTaylorProgram::integrate at order 3 over 4 step caps for N-dimensional families, near-point boxes.",
    );
    for (kind, family) in [
        ("linear chain", Spec::chain as fn(usize) -> Spec),
        ("dense linear", Spec::dense),
        ("Lorenz-96", Spec::lorenz96),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| {
                job(
                    &family(dim(m)),
                    Op::Integrate {
                        method: Method::Taylor(3),
                        steps: 4,
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "Taylor integrate by nonlinearity",
        "Van der Pol mu",
        "FlowTaylorProgram::integrate of Van der Pol over [0, 1/4] with step cap 1/16 as mu grows: stiffness forces halvings and eventually failure.",
    );
    let mus = ints(&[-4, -2, 0, 1, 2, 3, 4, 5, 6]);
    for order in [2usize, 4, 8] {
        atlas.sweep(
            &format!("order {order}"),
            &mus.iter().map(|&k| 2f64.powf(k)).collect::<Vec<_>>(),
            |m, _| {
                job(
                    &Spec::van_der_pol_mu(m.log2().round() as i32),
                    Op::Integrate {
                        method: Method::Taylor(order),
                        steps: 4,
                        step: Some(-4),
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "adaptive Taylor by horizon",
        "horizon (multiples of the Taylor step cap)",
        "AdaptiveTaylorField::integrate (orders 1..8, endpoint remainder limit 2^-24 per coordinate) over n step caps, near-point box; includes lazy program preparation.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Adaptive(8, -24),
                        steps: steps(m),
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "adaptive Taylor by remainder limit",
        "log2 remainder limit",
        "AdaptiveTaylorField::integrate (orders 1..10) over 4 step caps (Lorenz: 2) as the per-step remainder limit tightens: higher orders, then quality halvings.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &ints(&[-4, -8, -12, -16, -24, -32, -40, -48, -56]),
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Adaptive(10, m as i32),
                        steps: short(spec),
                        step: None,
                        width: near_point,
                    },
                )
            },
            call,
        );
    }
    atlas.grid(
        "Van der Pol",
        ("log2 remainder limit", &ints(&[-8, -16, -24, -32, -40])),
        ("horizon (step caps)", &powers_of_two(0, 6)),
        |limit, n, _| {
            job(
                &vdp,
                Op::Integrate {
                    method: Method::Adaptive(10, limit as i32),
                    steps: steps(n),
                    step: None,
                    width: near_point,
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "affine-noise integrate_noise by horizon",
        "horizon (multiples of the Taylor step cap)",
        "FlowTaylorProgram::integrate_noise at order 4 (correlated affine endpoints, 16 columns) over n step caps from a box of width 2^-6.",
    );
    for spec in [&rotation, &lv, &vdp, &lorenz] {
        atlas.sweep(
            &spec.name,
            &horizons,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Noise(4, 16),
                        steps: steps(m),
                        step: None,
                        width: noise_width,
                    },
                )
            },
            call,
        );
    }
    atlas.fuzz(
        "Lotka-Volterra, random center and width",
        &powers_of_two(0, 5),
        |m, rng| {
            job(
                &lv.jitter(rng, 1.0),
                Op::Integrate {
                    method: Method::Noise(4, 16),
                    steps: steps(m),
                    step: None,
                    width: random_width(rng, -12, -4),
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "affine-noise integrate_noise by column cap",
        "maximum retained noise columns",
        "FlowTaylorProgram::integrate_noise at order 3 over 4 step caps (Lorenz: 2) from a box of width 2^-6, by retained-column capacity (0 folds everything into residuals).",
    );
    let lorenz96 = Spec::lorenz96(4);
    let caps = ints(&[0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32]);
    for spec in [&vdp, &lorenz, &lorenz96] {
        atlas.sweep(
            &spec.name,
            &caps,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Noise(3, m as usize),
                        steps: short(spec),
                        step: None,
                        width: noise_width,
                    },
                )
            },
            call,
        );
    }
    atlas.grid(
        "Lorenz-96 n=4",
        (
            "maximum retained noise columns",
            &ints(&[0, 2, 4, 8, 16, 32]),
        ),
        ("horizon (step caps)", &powers_of_two(0, 4)),
        |cap, n, _| {
            job(
                &lorenz96,
                Op::Integrate {
                    method: Method::Noise(3, cap as usize),
                    steps: steps(n),
                    step: None,
                    width: noise_width,
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "affine-noise integrate_noise by initial width",
        "log2 state-box width",
        "FlowTaylorProgram::integrate_noise at order 4 (16 columns) over 4 step caps (Lorenz: 2) for boxes of width 2^k; compare with the Cartesian width sweep.",
    );
    for spec in evolving {
        atlas.sweep(
            &spec.name,
            &widths,
            |m, _| {
                job(
                    spec,
                    Op::Integrate {
                        method: Method::Noise(4, 16),
                        steps: short(spec),
                        step: None,
                        width: width(m),
                    },
                )
            },
            call,
        );
    }
    atlas.grid(
        "linear rotation, correlated",
        ("log2 state-box width", &grid_widths),
        ("horizon (step caps)", &grid_horizons),
        |w, n, _| {
            job(
                &rotation,
                Op::Integrate {
                    method: Method::Noise(4, 16),
                    steps: steps(n),
                    step: None,
                    width: Some(w as i32),
                },
            )
        },
        call,
    );

    atlas.operation(
        "evolution",
        "method comparison: rotation of a wide box",
        "horizon (multiples of 1/8)",
        "Linear rotation of a box of width 1/4 over n eighths of a time unit by each integrator (Picard at step 1/64, the others at 1/8). Cartesian boxes wrap (widen) but complete; correlated noise and QR frames pay per step to keep the set tight.",
    );
    let wide = Some(-2);
    for (kind, method, step) in [
        ("Picard (Cartesian)", Method::Picard, -6),
        ("Taylor order 4 (Cartesian)", Method::Taylor(4), -3),
        ("affine noise order 4", Method::Noise(4, 16), -3),
        ("QR frames order 4", Method::Qr(4, EVERY), -3),
    ] {
        atlas.sweep(
            kind,
            &horizons,
            |m, _| {
                let n = steps(m) << (-3 - step);
                job(
                    &rotation,
                    Op::Integrate {
                        method,
                        steps: n,
                        step: Some(step),
                        width: wide,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "method comparison: Van der Pol",
        "horizon (multiples of 1/16)",
        "Van der Pol from a box of width 2^-6 over n sixteenths of a time unit by each whole-horizon integrator at order 4 (adaptive: orders 1..8, limit 2^-24). Shows which method reaches longer horizons and at what cost.",
    );
    for (kind, method) in [
        ("Taylor order 4 (Cartesian)", Method::Taylor(4)),
        ("adaptive Taylor", Method::Adaptive(8, -24)),
        ("affine noise order 4", Method::Noise(4, 16)),
        ("QR frames order 4", Method::Qr(4, EVERY)),
    ] {
        atlas.sweep(
            kind,
            &horizons,
            |m, _| {
                job(
                    &vdp,
                    Op::Integrate {
                        method,
                        steps: steps(m),
                        step: Some(-4),
                        width: noise_width,
                    },
                )
            },
            call,
        );
    }

    atlas.operation(
        "evolution",
        "QR-frame integrate_qr by horizon",
        "horizon (multiples of the Taylor step cap)",
        "CoordinateField::integrate_qr at order 4 (one frame selection per 4 step caps) from the identity chart, box width 2^-6. Series named by system alone compare frames in every window; \"backoff 16\" series defer proposals after retained wins (QrProposalPolicy::Backoff, runs of at most 16 windows).",
    );
    for spec in [&rotation, &vdp, &lv, &lorenz] {
        for (suffix, proposals) in [("", EVERY), (", backoff 16", BACKOFF)] {
            atlas.sweep(
                &format!("{}{suffix}", spec.name),
                &horizons,
                |m, _| {
                    job(
                        spec,
                        Op::Integrate {
                            method: Method::Qr(4, proposals),
                            steps: steps(m),
                            step: None,
                            width: noise_width,
                        },
                    )
                },
                call,
            );
        }
    }

    atlas.operation(
        "evolution",
        "rational-time integrate by horizon",
        "horizon n (physical span [0, n/3])",
        "RationalTimeField::integrate: Picard evolution of the clock-normalized field over the exact rational span [0, n/3], near-point box, physical steps at most the Picard step cap. Van der Pol needs dozens of exact Picard steps and stops on endpoint mantissa growth.",
    );
    for spec in [&decay, &rotation, &vdp] {
        atlas.sweep(
            &spec.name,
            &powers_of_two(0, if quick { 6 } else { 9 }),
            |m, _| job(spec, Op::RationalTime { steps: steps(m) }),
            call,
        );
    }

    // ----- reachability -----
    atlas.operation(
        "reachability",
        "reach by initial width",
        "log2 state-box width",
        "ExpressionField::reach: certified endpoint cover within epsilon = 1/16 by exact initial-box subdivision, over 4 Picard step caps.",
    );
    for spec in [&decay, &rotation, &riccati, &vdp] {
        atlas.sweep(
            &spec.name,
            &ints(&[-24, -16, -12, -10, -8, -7, -6, -5, -4, -3, -2]),
            |m, _| {
                job(
                    spec,
                    Op::Reach {
                        steps: 4,
                        width: width(m),
                        epsilon: -4,
                    },
                )
            },
            call,
        );
    }
    atlas.fuzz(
        "Van der Pol, random center",
        &ints(&[-12, -8, -6, -4]),
        |m, rng| {
            job(
                &vdp.jitter(rng, 1.0),
                Op::Reach {
                    steps: 4,
                    width: width(m),
                    epsilon: -4,
                },
            )
        },
        call,
    );

    atlas.operation(
        "reachability",
        "reach by epsilon",
        "log2 epsilon",
        "ExpressionField::reach over 4 Picard step caps from a box of width 2^-6 as the thickening bound epsilon tightens.",
    );
    for spec in [&decay, &rotation, &riccati, &vdp] {
        atlas.sweep(
            &spec.name,
            &ints(&[0, -2, -4, -6, -8, -10, -12, -16, -20]),
            |m, _| {
                job(
                    spec,
                    Op::Reach {
                        steps: 4,
                        width: Some(-6),
                        epsilon: m as i32,
                    },
                )
            },
            call,
        );
    }

    // ----- coordinates -----
    atlas.operation(
        "coordinates",
        "in_coordinates by dimension",
        "state dimension",
        "ExpressionField::in_coordinates: certify an exact bidiagonal chart and its inverse, and construct the transformed field.",
    );
    for (kind, family) in [
        ("linear chain", Spec::chain as fn(usize) -> Spec),
        ("dense linear", Spec::dense),
        ("Lorenz-96", Spec::lorenz96),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| job(&family(dim(m)), Op::InCoordinates),
            call,
        );
    }

    atlas.operation(
        "coordinates",
        "in_qr_coordinates by dimension",
        "state dimension",
        "ExpressionField::in_qr_coordinates: exact QR of a lower-triangular all-ones basis (irrational square roots), then the transformed field.",
    );
    for (kind, family) in [
        ("linear chain", Spec::chain as fn(usize) -> Spec),
        ("dense linear", Spec::dense),
        ("Lorenz-96", Spec::lorenz96),
    ] {
        atlas.sweep(kind, &dims, |m, _| job(&family(dim(m)), Op::InQr), call);
    }

    atlas.operation(
        "coordinates",
        "chart transition by dimension",
        "state dimension",
        "CoordinateField::transition_to between the identity and a bidiagonal chart, and enclosing a box (width 2^-8) through a prepared transition.",
    );
    for (kind, enclose) in [
        ("prepare transition_to", false),
        ("enclose through transition", true),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| job(&Spec::lorenz96(dim(m)), Op::Transition { enclose }),
            call,
        );
    }

    atlas.finish();
}
