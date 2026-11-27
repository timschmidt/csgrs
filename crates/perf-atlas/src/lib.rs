//! Seeded wall-clock probes and JSON report assembly for crate performance atlases.
//!
//! Call Atlas::operation, then sweep, fuzz, or grid to measure callback-defined
//! work across magnitude axes and labeled input recipes. The report carries timings
//! and coarse callback statuses, not independent correctness, certainty, provenance,
//! or completed-workload evidence. The renderer in scripts/perf-atlas consumes the
//! perf-atlas/1 schema.
//!
//! One initial call estimates a fixed batch size; subsequent batches are not extended
//! until they reach the target. Only the initial result is checked through Outcome.
//! Fresh input objects are built outside each timed batch by default, but factories
//! may share state or caches; no cache reset or input-equivalence proof is provided.
//! Reuse mode retains one separate input for the repeated batches after the probe.
//!
//! Ceilings are checked after the initial call, and point budgets between batches.
//! They do not preempt callbacks, bound input generation, or impose aggregate work,
//! memory, or cancellation limits. Timing-based stopping can change which seeded
//! samples are reached. Only unwinding panics inside measurement are caught.
//!
//! Compare equivalent completed workloads under matched host/toolchain/settings;
//! report metadata alone does not establish those conditions. Output is assembled
//! in memory and written directly to a file or stdout, without atomic publication.
use std::fmt::Write as _;
use std::hint::black_box;
use std::time::{Duration, Instant};

/// Timing effort and sample-selection settings, accepted without validation.
///
/// Use full or quick for nonzero sample counts. A zero samples value can reach an
/// empty statistics vector and panic inside measurement. Durations guide calibration
/// and stopping; they are not hard timeouts or a total resource contract.
#[derive(Clone, Debug)]
pub struct Config {
    /// Target duration used once to estimate calls per batch; not a minimum measured batch duration.
    pub target_batch: Duration,
    /// Maximum repeated timing batches per point; zero is not validated and can cause a statistics panic.
    pub samples: usize,
    /// Elapsed-time threshold checked between batches, including input setup; not a preemptive deadline.
    pub point_budget: Duration,
    /// Initial callback duration threshold checked after completion; later calls are not individually checked.
    pub single_ceiling: Duration,
    /// Maximum independently seeded samples attempted at each fuzz magnitude before stopping rules.
    pub fuzz_per_magnitude: usize,
    /// Base state hashed with operation/kind names; reproducibility also depends on generators and stopping decisions.
    pub seed: u64,
    /// Case-sensitive substring required in operation names; None or an empty string selects every name.
    pub filter: Option<String>,
    /// Reported mode flag; setting it alone does not change any timing settings.
    pub quick: bool,
}

impl Config {
    /// Return report defaults: 2 ms target, seven samples, 250 ms point budget, and 2 s ceiling.
    ///
    /// Requests 24 fuzz inputs per magnitude, the fixed base seed, no filter, and quick=false.
    /// These are effort settings, not guarantees that every sample or magnitude executes.
    pub fn full() -> Self {
        Self {
            target_batch: Duration::from_millis(2),
            samples: 7,
            point_budget: Duration::from_millis(250),
            single_ceiling: Duration::from_secs(2),
            fuzz_per_magnitude: 24,
            seed: 0x5eed_a71a_5000_0001,
            filter: None,
            quick: false,
        }
    }

    /// Return smoke settings with the same seed/filter defaults and reduced timing effort.
    ///
    /// Uses 200 µs target, three samples, 25 ms point budget, 250 ms ceiling, six fuzz
    /// inputs per magnitude, and quick=true. It does not shorten caller-supplied axes.
    pub fn quick() -> Self {
        Self {
            target_batch: Duration::from_micros(200),
            samples: 3,
            point_budget: Duration::from_millis(25),
            single_ceiling: Duration::from_millis(250),
            fuzz_per_magnitude: 6,
            quick: true,
            ..Self::full()
        }
    }
}

/// Deterministic xorshift64* stream for fixture generation and reproducible substreams.
///
/// Sampling helpers use bit extraction or remainder reduction rather than a general
/// unbiased-distribution algorithm. Cloning preserves state; zero seeds are mapped
/// to one. The generator does not validate geometric fixtures built from its output.
#[derive(Clone, Debug)]
pub struct Rng(
    /// Nonzero xorshift state before output multiplication; Clone copies the exact stream position.
    u64,
);

impl Rng {
    /// Start at max(seed,1), making zero and one select the same deterministic stream.
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    /// Advance xorshift state by shifts 12,25,27, then return its wrapping star-multiplied output.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    /// Return the next word modulo max(bound,1), advancing state even when the result must be zero.
    ///
    /// A zero bound behaves like one. Remainder reduction is not an exact-uniformity
    /// proof for arbitrary bounds; this helper performs no rejection sampling.
    pub fn below(&mut self, bound: u64) -> u64 {
        self.next_u64() % bound.max(1)
    }
    /// Add a remainder-reduced offset to lo using wrapping signed arithmetic.
    ///
    /// For ordered endpoints other than the full i64 range, the span is hi-lo+1 and
    /// the result lies in that inclusive interval. Reversed endpoints are not rejected;
    /// the absolute difference still defines a forward span from lo. Full-range span
    /// addition saturates, excluding i64::MAX. Uniformity is not guaranteed.
    pub fn range(&mut self, lo: i64, hi: i64) -> i64 {
        let span = hi.abs_diff(lo).saturating_add(1);
        lo.wrapping_add(self.below(span) as i64)
    }
    /// Return whether the next generated word has its low bit set, consuming one word.
    pub fn coin(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }
    /// Map the upper 53 bits of the next word to an exactly representable multiple of 2^-53 in [0,1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Generate little-endian u32 limbs of a positive integer with max(bits,1) magnitude bits.
    ///
    /// Draws one word per limb, retains its low 32 bits, masks excess top bits, and forces
    /// the highest requested bit. A request for zero therefore returns the integer one.
    /// Allocation and generation scale with the requested width without a separate budget.
    pub fn limbs(&mut self, bits: u32) -> Vec<u32> {
        let bits = bits.max(1);
        let count = bits.div_ceil(32) as usize;
        let mut limbs: Vec<u32> = (0..count).map(|_| self.next_u64() as u32).collect();
        let top = (bits - 1) % 32;
        let last = limbs.last_mut().expect("at least one limb");
        *last &= if top == 31 {
            u32::MAX
        } else {
            (1u32 << (top + 1)) - 1
        };
        *last |= 1u32 << top;
        limbs
    }
    /// Format a newly generated positive limb integer as decimal ASCII without leading zeros.
    ///
    /// Uses limbs(bits), so bits=0 means one bit. Repeatedly divides the entire limb
    /// vector by ten, collects remainders, then reverses them. It does not trim high zero
    /// limbs between divisions; no conversion-work or memory bound is enforced.
    pub fn decimal(&mut self, bits: u32) -> String {
        let mut limbs = self.limbs(bits);
        let mut digits = Vec::new();
        while limbs.iter().any(|&l| l != 0) {
            let mut remainder = 0u64;
            for limb in limbs.iter_mut().rev() {
                let value = (remainder << 32) | u64::from(*limb);
                *limb = (value / 10) as u32;
                remainder = value % 10;
            }
            digits.push(b'0' + remainder as u8);
        }
        digits.reverse();
        String::from_utf8(digits).expect("ascii digits")
    }
}

/// Callback-defined initial-probe status used for atlas error marking.
///
/// Only the first result of a measurement is inspected; later repeated results are
/// black-boxed and dropped. Presence-based implementations do not recursively inspect
/// payloads: Some(Err(...)) and Ok(false) still succeed. Numeric and String results
/// always succeed, including nonfinite floats. Choose a callback return contract that
/// actually checks the intended completion condition and preserves needed evidence.
pub trait Outcome {
    /// Report whether this callback result satisfies the caller's completion contract; the harness checks only its first probe.
    fn succeeded(&self) -> bool;
    /// Default to no failure explanation; implementations may provide one for the initial failed probe.
    fn reason(&self) -> Option<String> {
        None
    }
}

/// Result alias for callbacks that supply a textual failure reason.
///
/// Err is marked as failure and its Debug representation is truncated for the report.
/// Ok succeeds regardless of its payload; this alias does not certify that payload.
pub type Checked<T> = Result<T, String>;

/// Keep at most 160 initial UTF-8 bytes on a character boundary, appending an ellipsis if truncated.
///
/// Returns short input unchanged. The ellipsis adds three bytes, so the returned
/// string can be 163 bytes. This does not bound the allocation of the original text.
fn short(text: String) -> String {
    /// Maximum retained prefix bytes before the three-byte ellipsis is appended.
    const LIMIT: usize = 160;
    if text.len() <= LIMIT {
        return text;
    }
    let mut end = LIMIT;
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    format!("{}…", &text[..end])
}
impl<T> Outcome for Option<T> {
    /// Accept Some without inspecting the contained value; None reports failure.
    fn succeeded(&self) -> bool {
        self.is_some()
    }
}
impl<T, E: std::fmt::Debug> Outcome for Result<T, E> {
    /// Accept Ok without inspecting its payload; Err reports failure.
    fn succeeded(&self) -> bool {
        self.is_ok()
    }
    /// Return a shortened Debug-formatted Err payload, allocating that formatting before truncation; Ok has no reason.
    fn reason(&self) -> Option<String> {
        self.as_ref().err().map(|e| short(format!("{e:?}")))
    }
}
impl Outcome for bool {
    /// Use the Boolean value itself as the status.
    fn succeeded(&self) -> bool {
        *self
    }
}
/// Implement unconditional Outcome success for the listed scalar/string/unit types, without validating their values.
macro_rules! always_succeeds {
    ($($t:ty),*) => {$(impl Outcome for $t {
        /// Always accept this value without validation, including nonfinite floats where applicable.
        fn succeeded(&self) -> bool { true }
    })*};
}
always_succeeds!(
    (),
    u8,
    u16,
    u32,
    u64,
    u128,
    usize,
    i8,
    i16,
    i32,
    i64,
    i128,
    isize,
    f32,
    f64,
    String
);
impl<A: Outcome, B: Outcome> Outcome for (A, B) {
    /// Require both tuple members to succeed, short-circuiting before the second when the first fails.
    fn succeeded(&self) -> bool {
        self.0.succeeded() && self.1.succeeded()
    }
    /// Return the first available member reason, preferring the left, without checking which member failed.
    fn reason(&self) -> Option<String> {
        self.0.reason().or_else(|| self.1.reason())
    }
}
impl<T: Outcome> Outcome for Vec<T> {
    /// Require every element to succeed, stopping at the first failure; the empty vector succeeds.
    fn succeeded(&self) -> bool {
        self.iter().all(Outcome::succeeded)
    }
    /// Return the first available element reason, without independently checking that element's status.
    fn reason(&self) -> Option<String> {
        self.iter().find_map(Outcome::reason)
    }
}

/// One axis sample's timing statistics and coarse first-probe status.
///
/// Default is an empty zero-valued record, not a performed measurement. Skipped and
/// panicked records use NaN timings, serialized as null. This carrier contains no
/// complete callback output, exactness receipt, or independent correctness witness.
#[derive(Clone, Debug, Default)]
struct Point {
    /// Caller-supplied axis value; nonfinite values become JSON null.
    magnitude: f64,
    /// Upper median per-call nanoseconds across batches, or the initial-call duration on a single/ceiling path.
    median_ns: f64,
    /// Smallest batch-average duration, or the initial duration; NaN for unmeasured/panicked records.
    min_ns: f64,
    /// Largest batch-average duration, or the initial duration; NaN for unmeasured/panicked records.
    max_ns: f64,
    /// Calls per repeated batch, not total calls; one for initial-only records and zero when skipped/panicked.
    iterations: u64,
    /// One of ok, single, error, ceiling, predicted, or skipped for emitted records; Default is empty.
    status: &'static str,
    /// Initial Outcome reason or truncated unwind description when retained; absent for ceiling/skipped records.
    reason: Option<String>,
}

/// Ordered timing records for one caller-supplied input-kind label; repeated magnitudes are allowed.
#[derive(Clone, Debug)]
struct Series {
    /// Caller label used for display and seeded stream selection.
    kind: String,
    /// Samples in generation order; fuzz may contain several entries with the same magnitude.
    points: Vec<Point>,
}

/// Row-major timing and error matrices with copied magnitude axes.
///
/// Rows correspond to b and columns to a. Missing cells conflate prediction, prior
/// blocking, and panics; errors distinguishes panics/failed probes where available.
/// No per-cell reason, iteration count, or timing spread is retained.
#[derive(Clone, Debug)]
struct Grid {
    /// Caller label for this grid recipe.
    kind: String,
    /// Display name of the horizontal/column axis.
    a_label: String,
    /// Display name of the vertical/row axis.
    b_label: String,
    /// Copied column magnitudes in caller order, without sorting or validation.
    a: Vec<f64>,
    /// Copied row magnitudes in caller order, without sorting or validation.
    b: Vec<f64>,
    /// Optional per-call nanoseconds for each row/column; ceiling durations remain present.
    ns: Vec<Vec<Option<f64>>>,
    /// Parallel matrix marking failed probes or caught panics; skipped/ceiling cells are false.
    errors: Vec<Vec<bool>>,
}

/// Report metadata plus the sweep, seeded-sample, and grid results registered under one name.
///
/// Names need not be unique. Group, descriptions, axis labels, and kind labels are
/// caller claims; the harness neither validates their mathematical meaning nor
/// checks that differently labeled series measure equivalent work.
#[derive(Clone, Debug)]
struct Operation {
    /// Operation identifier used for filtering and hashing; duplicates are allowed.
    name: String,
    /// Caller-defined display group, excluded from RNG hashing.
    group: String,
    /// Caller-supplied workload explanation, serialized without checking its claims.
    description: String,
    /// Display label for one-dimensional sweep magnitudes.
    magnitude: String,
    /// Ordinary sweeps in registration order.
    series: Vec<Series>,
    /// Independently seeded sample series in registration order.
    fuzz: Vec<Series>,
    /// Two-axis measurements in registration order.
    grids: Vec<Grid>,
}

/// Accumulate callback measurements for one crate and serialize their report.
///
/// Registration order controls report order. Each measurement method operates on
/// the most recently declared operation and applies the configured name filter.
/// Changing reuse affects later measurements only. No thread worker, process-level
/// timeout, output-size limit, or semantic validator is owned by this collector.
pub struct Atlas {
    /// Report crate label; it does not participate in RNG hashing.
    crate_name: String,
    /// Unvalidated settings used by subsequent measurement calls.
    config: Config,
    /// Direct output-file destination, or stdout when absent; parent directories are not created.
    output: Option<String>,
    /// All declarations, including those with no selected measurements.
    operations: Vec<Operation>,
    /// Construction timestamp used for progress and total elapsed metadata.
    started: Instant,
    /// Whether repeated batches share one input; the initial probe always uses a separate input.
    reuse: bool,
}

/// Maximum simultaneously constructed inputs in one fresh batch; not a byte or total-work limit.
const MAX_FRESH: usize = 4096;

/// Multiplier over the initial-call ceiling used by the two-sample growth heuristic.
const PREDICTED_FACTOR: f64 = 8.0;

/// Internal measurement result separating recorded statistics, an exceeded initial ceiling, and unwind failure.
enum Measured {
    /// Completed timing record, which can still have an error status from its initial probe.
    Done(
        /// Recorded first-call or batch statistics, before the caller assigns the axis magnitude.
        Point,
    ),
    /// Initial callback completed after the configured single-call ceiling.
    TooSlow(
        /// Initial callback duration in nanoseconds; its Outcome status and reason are discarded.
        f64,
    ),
    /// Unwinding panic during measurement; this is not a typed kernel outcome or transactional rollback.
    Panicked(
        /// Truncated panic description, including the panic prefix.
        String,
    ),
}

/// Describe an unwind payload as a shortened panic-prefixed string.
///
/// Copies str/String payload text; other payloads use a fixed fallback description.
/// Formatting and the original panic allocation are not bounded by the later truncation.
fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    let text = payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "non-string panic payload".to_owned());
    short(format!("panic: {text}"))
}

impl Atlas {
    /// Parse process arguments into atlas settings and an optional output path.
    ///
    /// Recognizes --quick, --out PATH, --filter TEXT, and --seed U64; --bench and
    /// --nocapture are ignored. Unknown arguments panic. Missing option values become
    /// None and an invalid seed falls back to the full/quick default. Repeated options
    /// use their latest parsed value. No report file is opened until finish.
    pub fn from_args(crate_name: &str) -> Self {
        let mut config = Config::full();
        let mut output = None;
        let mut args = std::env::args().skip(1);
        let mut filter = None;
        let mut seed = None;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--quick" => {
                    config = Config {
                        filter: config.filter.clone(),
                        ..Config::quick()
                    }
                }
                "--out" => output = args.next(),
                "--filter" => filter = args.next(),
                "--seed" => seed = args.next().and_then(|s| s.parse().ok()),
                // Arguments that `cargo bench`/test harnesses may pass through.
                "--bench" | "--nocapture" => {}
                other => panic!("unknown perf-atlas argument: {other}"),
            }
        }
        config.filter = filter;
        if let Some(seed) = seed {
            config.seed = seed;
        }
        Self::new(crate_name, config, output)
    }

    /// Create an empty atlas with supplied unvalidated settings, no input reuse, and a fresh elapsed-time origin.
    pub fn new(crate_name: &str, config: Config, output: Option<String>) -> Self {
        Self {
            crate_name: crate_name.to_owned(),
            config,
            output,
            operations: Vec::new(),
            started: Instant::now(),
            reuse: false,
        }
    }

    /// Choose whether later repeated timing batches share one factory-produced input.
    ///
    /// The initial probe always uses a separate input. With reuse=false, each repeated
    /// call gets a factory invocation outside timing; with true, one further invocation
    /// supplies all repeated batches. This flag neither resets caches nor verifies that
    /// the factory creates independent equal-valued inputs. Fuzz/grid inherit it too.
    pub fn reuse_inputs(&mut self, reuse: bool) {
        self.reuse = reuse;
    }

    /// Borrow the settings stored in this atlas; no mutation or validation occurs.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Hash the configured seed with operation bytes, a zero separator, and kind bytes into a stream.
    ///
    /// Uses wrapping FNV-style multiplication and maps a final zero state to one.
    /// Crate/group names are excluded. Matching inputs give matching starting states;
    /// different labels need not be collision-free, and later sample selection can depend
    /// on timing-based stops. Repeated calls restart the same labeled stream.
    pub fn rng(&self, operation: &str, kind: &str) -> Rng {
        let mut hash = self.config.seed;
        for byte in operation.bytes().chain([0]).chain(kind.bytes()) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
        }
        Rng::new(hash)
    }

    /// Apply the case-sensitive operation-name substring filter; None and an empty string both select all names.
    fn selected(&self, operation: &str) -> bool {
        self.config
            .filter
            .as_ref()
            .is_none_or(|filter| operation.contains(filter.as_str()))
    }

    /// Append operation metadata and make it current for following measurements.
    ///
    /// Copies all labels/descriptions and prints elapsed progress to stderr even when
    /// the configured filter will reject this name. Duplicate names are accepted and
    /// share stream seeds for equal kind labels; no prior declaration is replaced.
    pub fn operation(&mut self, group: &str, name: &str, magnitude: &str, description: &str) {
        self.operations.push(Operation {
            name: name.to_owned(),
            group: group.to_owned(),
            description: description.to_owned(),
            magnitude: magnitude.to_owned(),
            series: Vec::new(),
            fuzz: Vec::new(),
            grids: Vec::new(),
        });
        eprintln!(
            "[{:>7.1}s] {group}/{name}",
            self.started.elapsed().as_secs_f64()
        );
    }

    /// Borrow the latest operation, panicking if no operation has been declared.
    fn current(&mut self) -> &mut Operation {
        self.operations
            .last_mut()
            .expect("declare an operation before sweeping it")
    }

    /// Catch unwinding panics around the complete unguarded measurement attempt.
    ///
    /// Includes input construction, callback execution, Outcome inspection, statistics,
    /// and ordinary drops in the protected scope. It does not catch aborts, interrupt
    /// nontermination, roll back external effects, or reset shared caches after a panic.
    /// A second panic during unwinding can still abort. The payload is shortened for reporting.
    fn measure<I, R: Outcome>(
        config: &Config,
        reuse: bool,
        copy: &mut dyn FnMut() -> I,
        run: &mut dyn FnMut(&I) -> R,
    ) -> Measured {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            Self::measure_unguarded(config, reuse, copy, run)
        }))
        .unwrap_or_else(|payload| Measured::Panicked(panic_message(payload)))
    }

    /// Probe once, estimate a fixed batch size, and collect batch-average wall-clock durations.
    ///
    /// The initial input is built before timing. Only run is timed for the probe;
    /// Outcome inspection and result/input destruction follow. A probe over the ceiling
    /// returns TooSlow even if its status failed. Otherwise, if twice its duration exceeds
    /// the point budget, returns a single record with its status and optional reason.
    ///
    /// Calls per batch are ceil(target/probe_ns), with probe_ns floored at one and the
    /// count clamped to 1..MAX_FRESH, or 1..(usize::MAX>>1) in reuse mode. No later
    /// calibration ensures the target is attained. Fresh batches build all inputs first;
    /// reuse mode builds one separate shared input after the probe. Repeated timing
    /// includes loop/black_box overhead and result drops, but excludes input setup/drop.
    /// Repeated results are not checked with Outcome and can disagree with the probe.
    ///
    /// Checks elapsed time, including setup, only between batches; if samples>0, at
    /// least one batch runs even after setup consumed the point budget. The sorted
    /// batch means supply min, max, and upper median; iterations records calls per batch,
    /// not total calls. samples=0 can panic on empty statistics. No call timeout, total
    /// work/memory bound, cancellation, or independent result validation is supplied.
    fn measure_unguarded<I, R: Outcome>(
        config: &Config,
        reuse: bool,
        copy: &mut dyn FnMut() -> I,
        run: &mut dyn FnMut(&I) -> R,
    ) -> Measured {
        let start = Instant::now();
        let input = copy();
        let t = Instant::now();
        let result = run(black_box(&input));
        let first = t.elapsed();
        let succeeded = result.succeeded();
        let reason = if succeeded { None } else { result.reason() };
        black_box(result);
        drop(input);
        let status = if succeeded { "ok" } else { "error" };
        if first > config.single_ceiling {
            return Measured::TooSlow(first.as_nanos() as f64);
        }
        if first * 2 > config.point_budget {
            let ns = first.as_nanos() as f64;
            return Measured::Done(Point {
                magnitude: 0.0,
                median_ns: ns,
                min_ns: ns,
                max_ns: ns,
                iterations: 1,
                status: if succeeded { "single" } else { "error" },
                reason,
            });
        }
        let first_ns = first.as_nanos().max(1) as f64;
        let wanted = (config.target_batch.as_nanos() as f64 / first_ns).ceil() as usize;
        let calls = wanted.clamp(1, if reuse { usize::MAX >> 1 } else { MAX_FRESH });
        let mut samples = Vec::new();
        let shared = if reuse { Some(copy()) } else { None };
        while samples.len() < config.samples
            && (samples.is_empty() || start.elapsed() < config.point_budget)
        {
            let ns = if let Some(input) = &shared {
                let t = Instant::now();
                for _ in 0..calls {
                    black_box(run(black_box(input)));
                }
                t.elapsed().as_nanos()
            } else {
                let inputs: Vec<I> = (0..calls).map(|_| copy()).collect();
                let t = Instant::now();
                for input in &inputs {
                    black_box(run(black_box(input)));
                }
                let ns = t.elapsed().as_nanos();
                drop(inputs);
                ns
            };
            samples.push(ns as f64 / calls as f64);
        }
        samples.sort_by(f64::total_cmp);
        Measured::Done(Point {
            magnitude: 0.0,
            median_ns: samples[samples.len() / 2],
            min_ns: samples[0],
            max_ns: samples[samples.len() - 1],
            iterations: calls as u64,
            status,
            reason,
        })
    }

    /// Create an unmeasured point with NaN timings, zero iterations, the supplied status, and no reason.
    fn skipped(magnitude: f64, status: &'static str) -> Point {
        Point {
            magnitude,
            median_ns: f64::NAN,
            min_ns: f64::NAN,
            max_ns: f64::NAN,
            iterations: 0,
            status,
            reason: None,
        }
    }

    // Growth of the last two timed points predicts the next one; a prediction
    // far over the ceiling skips the call instead of waiting for it.
    /// Predict excessive next cost from the final two timings, without inspecting magnitudes.
    ///
    /// Uses b*max(b/a,1) > 8*single_ceiling and returns false with fewer than two samples.
    /// It assumes continued cost growth, not a proven complexity bound. Inputs are not
    /// validated for positivity/finiteness; floating division/comparison rules apply.
    fn predicted_too_slow(config: &Config, previous: &[f64]) -> bool {
        let [.., a, b] = previous else {
            return false;
        };
        let growth = (b / a).max(1.0);
        b * growth > config.single_ceiling.as_nanos() as f64 * PREDICTED_FACTOR
    }

    /// Measure one labeled recipe at each supplied magnitude until a ceiling, panic, or prediction stops it.
    ///
    /// Requires a current operation before applying the name filter. For each reached
    /// magnitude, seeds a substream and calls make with a cloned state on every factory
    /// invocation. Equal inputs require make to be deterministic beyond that RNG argument;
    /// external mutable state is not reset. Magnitudes are used in caller order.
    ///
    /// Ordinary failed Outcome probes retain timings and do not stop the series. Their
    /// finite timings also feed prediction. A ceiling stores the initial duration and
    /// discards its Outcome status/reason; a panic stores an error and stops. Prediction
    /// marks its first omitted point predicted; all subsequent points are skipped. An
    /// empty axis still appends an empty Series. No complete-axis or hard-timeout promise
    /// is made; measurement's setup, reuse, and resource limitations apply.
    pub fn sweep<I, R>(
        &mut self,
        kind: &str,
        magnitudes: &[f64],
        mut make: impl FnMut(f64, &mut Rng) -> I,
        mut run: impl FnMut(&I) -> R,
    ) where
        R: Outcome,
    {
        let name = self.current().name.clone();
        if !self.selected(&name) {
            return;
        }
        let mut rng = self.rng(&name, kind);
        let mut points = Vec::new();
        let mut stopped = false;
        let mut timed: Vec<f64> = Vec::new();
        for &magnitude in magnitudes {
            if stopped {
                points.push(Self::skipped(magnitude, "skipped"));
                continue;
            }
            if Self::predicted_too_slow(&self.config, &timed) {
                stopped = true;
                points.push(Self::skipped(magnitude, "predicted"));
                continue;
            }
            let state = Rng::new(rng.next_u64());
            let point = match Self::measure(
                &self.config,
                self.reuse,
                &mut || make(magnitude, &mut state.clone()),
                &mut run,
            ) {
                Measured::Done(point) => Point { magnitude, ..point },
                Measured::TooSlow(ns) => {
                    stopped = true;
                    Point {
                        magnitude,
                        median_ns: ns,
                        min_ns: ns,
                        max_ns: ns,
                        iterations: 1,
                        status: "ceiling",
                        reason: None,
                    }
                }
                Measured::Panicked(reason) => {
                    stopped = true;
                    Point {
                        reason: Some(reason),
                        ..Self::skipped(magnitude, "error")
                    }
                }
            };
            if point.median_ns.is_finite() {
                timed.push(point.median_ns);
            }
            points.push(point);
        }
        self.current().series.push(Series {
            kind: kind.to_owned(),
            points,
        });
    }

    /// Measure up to fuzz_per_magnitude independently seeded inputs per magnitude.
    ///
    /// Requires a current operation before applying the name filter; otherwise panics.
    /// Uses the fuzz:kind seed namespace, one batch sample, and quarter-sized target and
    /// point-budget durations. Factories clone each sample's state; this is deterministic
    /// seeded sampling, not a mutation-based fuzzing engine or correctness oracle.
    ///
    /// A ceiling or caught panic ends the current magnitude. Two consecutive magnitudes
    /// with either event end the series without placeholders for omitted magnitudes;
    /// an ordinary failed Outcome does not trigger this stop. A magnitude without such
    /// an event resets the counter. Zero fuzz_per_magnitude appends an empty series.
    /// Seed consumption depends on reached samples, and no inner call is preempted.
    pub fn fuzz<I, R>(
        &mut self,
        kind: &str,
        magnitudes: &[f64],
        mut make: impl FnMut(f64, &mut Rng) -> I,
        mut run: impl FnMut(&I) -> R,
    ) where
        R: Outcome,
    {
        let name = self.current().name.clone();
        if !self.selected(&name) {
            return;
        }
        let mut rng = self.rng(&name, &format!("fuzz:{kind}"));
        let config = Config {
            samples: 1,
            target_batch: self.config.target_batch / 4,
            point_budget: self.config.point_budget / 4,
            ..self.config.clone()
        };
        let mut points = Vec::new();
        let mut ceilings_in_a_row = 0;
        for &magnitude in magnitudes {
            if ceilings_in_a_row >= 2 {
                break;
            }
            let mut hit_ceiling = false;
            for _ in 0..self.config.fuzz_per_magnitude {
                let state = Rng::new(rng.next_u64());
                match Self::measure(
                    &config,
                    self.reuse,
                    &mut || make(magnitude, &mut state.clone()),
                    &mut run,
                ) {
                    Measured::Done(point) => points.push(Point { magnitude, ..point }),
                    Measured::TooSlow(ns) => {
                        points.push(Point {
                            magnitude,
                            median_ns: ns,
                            min_ns: ns,
                            max_ns: ns,
                            iterations: 1,
                            status: "ceiling",
                            reason: None,
                        });
                        hit_ceiling = true;
                        break;
                    }
                    Measured::Panicked(reason) => {
                        points.push(Point {
                            reason: Some(reason),
                            ..Self::skipped(magnitude, "error")
                        });
                        hit_ceiling = true;
                        break;
                    }
                }
            }
            ceilings_in_a_row = if hit_ceiling {
                ceilings_in_a_row + 1
            } else {
                0
            };
        }
        self.current().fuzz.push(Series {
            kind: kind.to_owned(),
            points,
        });
    }

    /// Measure rows from b and columns from a, propagating a stopping boundary through later rows.
    ///
    /// Requires a current operation before applying the name filter; otherwise panics.
    /// Uses grid:kind seeds, at most three samples, and half the configured point budget.
    /// Within a row, a prior block, ceiling, panic, or timing prediction stops that cell
    /// and all columns to its right; those columns stay blocked in subsequent rows.
    /// This assumes cost grows in caller axis order and does not validate that assumption.
    ///
    /// Decided/failed probe durations remain present, with error=true only for failed
    /// probes. A ceiling retains its duration with error=false, even if its probe failed;
    /// a caught panic stores None/error=true and discards its message. Skipped cells use
    /// None/error=false. Per-cell reasons, timing spread, and iteration counts are lost.
    /// Factories and clocks inherit measure's limitations, including zero-sample panics;
    /// no full-grid coverage, semantic validation, or hard timeout is established.
    pub fn grid<I, R>(
        &mut self,
        kind: &str,
        (a_label, a): (&str, &[f64]),
        (b_label, b): (&str, &[f64]),
        mut make: impl FnMut(f64, f64, &mut Rng) -> I,
        mut run: impl FnMut(&I) -> R,
    ) where
        R: Outcome,
    {
        let name = self.current().name.clone();
        if !self.selected(&name) {
            return;
        }
        let mut rng = self.rng(&name, &format!("grid:{kind}"));
        let config = Config {
            samples: self.config.samples.min(3),
            point_budget: self.config.point_budget / 2,
            ..self.config.clone()
        };
        let mut rows = Vec::new();
        let mut errors = Vec::new();
        // A column whose cell hit the ceiling (or was skipped) in one row is
        // skipped in the following rows too, assuming cost grows along both axes.
        let mut blocked = vec![false; a.len()];
        for &y in b {
            let mut row = Vec::new();
            let mut failed = Vec::new();
            let mut stopped = false;
            let mut timed: Vec<f64> = Vec::new();
            for (column, &x) in a.iter().enumerate() {
                if stopped || blocked[column] || Self::predicted_too_slow(&config, &timed) {
                    stopped = true;
                    blocked[column] = true;
                    row.push(None);
                    failed.push(false);
                    continue;
                }
                let state = Rng::new(rng.next_u64());
                match Self::measure(
                    &config,
                    self.reuse,
                    &mut || make(x, y, &mut state.clone()),
                    &mut run,
                ) {
                    Measured::Done(point) => {
                        row.push(Some(point.median_ns));
                        timed.push(point.median_ns);
                        failed.push(point.status == "error");
                    }
                    Measured::TooSlow(ns) => {
                        stopped = true;
                        blocked[column] = true;
                        row.push(Some(ns));
                        failed.push(false);
                    }
                    Measured::Panicked(_) => {
                        stopped = true;
                        blocked[column] = true;
                        row.push(None);
                        failed.push(true);
                    }
                }
            }
            rows.push(row);
            errors.push(failed);
        }
        self.current().grids.push(Grid {
            kind: kind.to_owned(),
            a_label: a_label.to_owned(),
            b_label: b_label.to_owned(),
            a: a.to_vec(),
            b: b.to_vec(),
            ns: rows,
            errors,
        });
    }

    /// Consume the atlas, assemble one perf-atlas/1 JSON document, and write it to the configured sink.
    ///
    /// Serializes operations with at least one series/fuzz/grid record, including empty
    /// records; declarations with none are omitted. Records OS, architecture, base seed,
    /// quick flag, elapsed time, and selected timing settings, but not toolchain, hardware,
    /// filter, or reuse mode. Thus the document alone is not a reproducibility manifest.
    ///
    /// Builds the complete JSON and intermediate strings in memory. File output directly
    /// creates/truncates the destination without creating parent directories or atomic
    /// replacement; I/O failures panic and can leave partial output. Stdout printing can
    /// also panic on failure. This is report serialization, not geometry/proof replay.
    pub fn finish(self) {
        let mut json = String::new();
        let quick = self.config.quick;
        let _ = write!(
            json,
            "{{\"format\":\"perf-atlas/1\",\"crate\":{},\"quick\":{quick},\"seed\":{},\"os\":{},\"arch\":{},\"elapsed_s\":{:.3},\"settings\":{{\"target_batch_ns\":{},\"samples\":{},\"point_budget_ns\":{},\"single_ceiling_ns\":{},\"fuzz_per_magnitude\":{}}},\"operations\":[",
            quote(&self.crate_name),
            self.config.seed,
            quote(std::env::consts::OS),
            quote(std::env::consts::ARCH),
            self.started.elapsed().as_secs_f64(),
            self.config.target_batch.as_nanos(),
            self.config.samples,
            self.config.point_budget.as_nanos(),
            self.config.single_ceiling.as_nanos(),
            self.config.fuzz_per_magnitude,
        );
        let operations: Vec<_> = self
            .operations
            .iter()
            .filter(|op| !op.series.is_empty() || !op.fuzz.is_empty() || !op.grids.is_empty())
            .collect();
        for (i, op) in operations.iter().enumerate() {
            if i > 0 {
                json.push(',');
            }
            let _ = write!(
                json,
                "{{\"name\":{},\"group\":{},\"description\":{},\"magnitude\":{},\"series\":[{}],\"fuzz\":[{}],\"grids\":[{}]}}",
                quote(&op.name),
                quote(&op.group),
                quote(&op.description),
                quote(&op.magnitude),
                op.series
                    .iter()
                    .map(series_json)
                    .collect::<Vec<_>>()
                    .join(","),
                op.fuzz
                    .iter()
                    .map(series_json)
                    .collect::<Vec<_>>()
                    .join(","),
                op.grids.iter().map(grid_json).collect::<Vec<_>>().join(","),
            );
        }
        json.push_str("]}\n");
        match self.output {
            Some(path) => {
                std::fs::write(&path, json).unwrap_or_else(|e| panic!("writing {path}: {e}"));
                eprintln!(
                    "perf-atlas: {} operations in {:.1}s -> {path}",
                    operations.len(),
                    self.started.elapsed().as_secs_f64()
                );
            }
            None => print!("{json}"),
        }
    }
}

/// Format finite binary64 values with Rust Display; map every NaN or infinity to JSON null.
fn number(value: f64) -> String {
    if value.is_finite() {
        format!("{value}")
    } else {
        "null".to_owned()
    }
}

/// Encode UTF-8 text as one JSON string, escaping quotes, backslashes, and control characters.
///
/// Newlines use the short escape; other characters below U+0020 use four-digit
/// hexadecimal escapes. Other Unicode characters remain literal. Allocation is not
/// bounded; the returned JSON literal is not an HTML/script-context escaping contract.
fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// Serialize a series using ordered seven-element point rows.
///
/// Row order is magnitude, median_ns, min_ns, max_ns, iterations, status, reason.
/// Nonfinite numbers and absent reasons become null. Builds per-point strings and
/// joins them; no schema, status, reason, or timing-consistency validation occurs.
fn series_json(series: &Series) -> String {
    let points: Vec<String> = series
        .points
        .iter()
        .map(|p| {
            format!(
                "[{},{},{},{},{},{},{}]",
                number(p.magnitude),
                number(p.median_ns),
                number(p.min_ns),
                number(p.max_ns),
                p.iterations,
                quote(p.status),
                p.reason.as_deref().map_or("null".to_owned(), quote)
            )
        })
        .collect();
    format!(
        "{{\"kind\":{},\"points\":[{}]}}",
        quote(&series.kind),
        points.join(",")
    )
}

/// Serialize grid axes, row-major optional durations, and the Boolean error matrix.
///
/// None and nonfinite durations both become null. Matrix dimensions are trusted;
/// no reason/status reconstruction or consistency validation occurs. Intermediate
/// strings are allocated for rows and joined into the returned object.
fn grid_json(grid: &Grid) -> String {
    let rows: Vec<String> = grid
        .ns
        .iter()
        .map(|row| {
            format!(
                "[{}]",
                row.iter()
                    .map(|v| v.map_or("null".to_owned(), number))
                    .collect::<Vec<_>>()
                    .join(",")
            )
        })
        .collect();
    format!(
        "{{\"kind\":{},\"a_label\":{},\"b_label\":{},\"a\":[{}],\"b\":[{}],\"ns\":[{}],\"errors\":[{}]}}",
        quote(&grid.kind),
        quote(&grid.a_label),
        quote(&grid.b_label),
        grid.a
            .iter()
            .map(|v| number(*v))
            .collect::<Vec<_>>()
            .join(","),
        grid.b
            .iter()
            .map(|v| number(*v))
            .collect::<Vec<_>>()
            .join(","),
        rows.join(","),
        grid.errors
            .iter()
            .map(|row| format!(
                "[{}]",
                row.iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            ))
            .collect::<Vec<_>>()
            .join(",")
    )
}

/// Return binary64 powers 2^lo through 2^hi in inclusive exponent order.
///
/// An inverted range is empty. Extreme exponents can underflow to zero or overflow
/// to infinity; no range, finiteness, allocation, or output-size check is applied.
pub fn powers_of_two(lo: i32, hi: i32) -> Vec<f64> {
    (lo..=hi).map(|e| 2f64.powi(e)).collect()
}

/// Build a rounded binary64 geometric ladder, then remove adjacent equal values.
///
/// Clamps both endpoints to at least one and requests at least two samples. Reversed
/// endpoints produce a descending recipe; equal endpoints deduplicate to one value.
/// The result can contain fewer than steps entries. Large u64 endpoints lose integer
/// precision on conversion, so exact endpoint preservation and exact integer ratios
/// are not promised. No sorting, work limit, or allocation bound is added.
pub fn ladder(lo: u64, hi: u64, steps: usize) -> Vec<f64> {
    let (lo, hi) = (lo.max(1) as f64, hi.max(1) as f64);
    let mut out: Vec<f64> = (0..steps.max(2))
        .map(|i| (lo * (hi / lo).powf(i as f64 / (steps.max(2) - 1) as f64)).round())
        .collect();
    out.dedup();
    out
}

/// Bounded harness regressions for generation, record assembly, fresh inputs, and unwind reporting.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Check eight seeded limb widths, the one-bit decimal identity, and a 64-bit decimal length floor.
    ///
    /// Does not test the zero-width convention, distribution, all decimal digits, or
    /// resource behavior; the fixed seed and width list are a bounded regression sample.
    fn limbs_have_the_requested_bit_length() {
        let mut rng = Rng::new(7);
        for bits in [1u32, 2, 31, 32, 33, 64, 100, 257] {
            let limbs = rng.limbs(bits);
            let top = limbs.len() as u32 * 32 - limbs.last().unwrap().leading_zeros();
            assert_eq!(top, bits);
        }
        assert_eq!(rng.decimal(1), "1");
        assert!(Rng::new(3).decimal(64).len() >= 19);
    }

    #[test]
    /// Exercise small sweep/fuzz/grid records, one JSON quote, fresh inputs, panic stopping, and a ladder.
    ///
    /// Asserts selected row counts, no repeated use of each fresh Cell, a formatted panic
    /// reason and following skipped point, and one four-value ladder. Despite the name,
    /// it does not call finish or parse a complete JSON document. Uses real clocks and
    /// quick settings, so severe scheduling delays can affect its stopping assumptions.
    /// No broader performance, success-receipt, or distribution claim follows.
    fn sweeps_record_points_and_json() {
        let mut atlas = Atlas::new("test", Config::quick(), None);
        atlas.operation("g", "add", "bits", "integer addition");
        atlas.sweep(
            "u64",
            &[1.0, 2.0],
            |m, rng| (m as u64, rng.next_u64()),
            |(a, b)| a.wrapping_add(*b),
        );
        atlas.fuzz("u64", &[1.0], |_, rng| rng.next_u64(), |a| a.count_ones());
        atlas.grid(
            "u64",
            ("a", &[1.0, 2.0]),
            ("b", &[1.0]),
            |a, b, _| (a, b),
            |(a, b)| a * b,
        );
        assert_eq!(atlas.operations[0].series[0].points.len(), 2);
        assert_eq!(
            atlas.operations[0].fuzz[0].points.len(),
            Config::quick().fuzz_per_magnitude
        );
        assert_eq!(atlas.operations[0].grids[0].ns.len(), 1);
        assert_eq!(quote("a\"b\n"), "\"a\\\"b\\n\"");

        // Every timed call gets its own input object with the same value.
        use std::cell::Cell;
        let built = Cell::new(0u64);
        let used = Cell::new(0u64);
        let mut atlas = Atlas::new("test", Config::quick(), None);
        atlas.operation("g", "fresh", "m", "fresh copies");
        atlas.sweep(
            "k",
            &[1.0],
            |_, rng| {
                built.set(built.get() + 1);
                (rng.next_u64(), Cell::new(false))
            },
            |(value, seen)| {
                assert!(!seen.replace(true), "an input object was timed twice");
                used.set(used.get() + 1);
                *value
            },
        );
        assert!(used.get() > 1 && built.get() >= used.get());

        // A panic is recorded as a failed point and ends the series.
        let mut atlas = Atlas::new("test", Config::quick(), None);
        atlas.operation("g", "panics", "m", "panics at m = 2");
        atlas.sweep(
            "k",
            &[1.0, 2.0, 3.0],
            |m, _| m,
            |m| if *m >= 2.0 { panic!("boom {m}") } else { *m },
        );
        let points = &atlas.operations[0].series[0].points;
        assert_eq!(points[1].status, "error");
        assert_eq!(points[1].reason.as_deref(), Some("panic: boom 2"));
        assert_eq!(points[2].status, "skipped");
        assert_eq!(ladder(1, 1000, 4), vec![1.0, 10.0, 100.0, 1000.0]);
    }
}
