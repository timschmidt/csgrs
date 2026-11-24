//! Performance atlas sweeps.
//!
//! A crate's `perf_atlas` example declares its operations. Each operation is
//! swept across input kinds (representations such as integer, rational or
//! irrational) and a magnitude axis (bit length, exponent, degree, dimension,
//! horizon). Two-argument operations can also be swept over a magnitude grid,
//! and seeded random ("fuzzed") inputs are timed one by one. The atlas writes
//! one JSON document, which `scripts/perf-atlas/render.py` turns into a
//! self-contained HTML report of charts.
//!
//! Timing is wall-clock with calibrated batches: each point repeats the
//! operation until a batch reaches a target duration, then records the median,
//! minimum and maximum per-operation time over several batches. Input
//! construction is outside the timer. A point whose single call exceeds the
//! configured ceiling ends that series; later magnitudes are recorded as
//! skipped so the report can show where an operation leaves its budget.
//!
//! These are focused probes on one host, not a cross-library claim. Compare
//! reports only when they come from the same host, toolchain and settings.
use std::fmt::Write as _;
use std::hint::black_box;
use std::time::{Duration, Instant};

/// Sweep effort. `quick` is for smoke runs and CI; the default is for reports.
#[derive(Clone, Debug)]
pub struct Config {
    /// Shortest batch accepted as a timing sample.
    pub target_batch: Duration,
    /// Batches recorded per point.
    pub samples: usize,
    /// Time allowed for one point, including calibration.
    pub point_budget: Duration,
    /// A single call longer than this ends the series.
    pub single_ceiling: Duration,
    /// Fuzzed inputs per magnitude.
    pub fuzz_per_magnitude: usize,
    /// Seed for every generated input.
    pub seed: u64,
    /// Only operations whose name contains this text run.
    pub filter: Option<String>,
    /// Quick mode flag, recorded in the output.
    pub quick: bool,
}

impl Config {
    /// Report settings.
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

    /// Smoke settings: the same sweeps with short batches and few samples.
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

/// Deterministic xorshift64* generator for fuzzed and swept inputs.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    /// A generator from a nonzero seed.
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    /// Next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    /// Uniform integer in `0..bound`, for a nonzero bound.
    pub fn below(&mut self, bound: u64) -> u64 {
        self.next_u64() % bound.max(1)
    }
    /// Uniform integer in `lo..=hi`.
    pub fn range(&mut self, lo: i64, hi: i64) -> i64 {
        let span = hi.abs_diff(lo).saturating_add(1);
        lo.wrapping_add(self.below(span) as i64)
    }
    /// Fair coin.
    pub fn coin(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }
    /// Uniform float in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Little-endian 32-bit limbs of a random integer with exactly `bits` bits
    /// (top bit set), for building big integers in any representation.
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
    /// Decimal digits of a random integer with exactly `bits` bits.
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

/// Whether a timed call produced its intended result. Points whose first call
/// fails (an error, `None` or `false`) are kept but marked, because a fast
/// refusal is not a fast successful operation.
pub trait Outcome {
    /// `true` when the call produced its intended result.
    fn succeeded(&self) -> bool;
}
impl<T> Outcome for Option<T> {
    fn succeeded(&self) -> bool {
        self.is_some()
    }
}
impl<T, E> Outcome for Result<T, E> {
    fn succeeded(&self) -> bool {
        self.is_ok()
    }
}
impl Outcome for bool {
    fn succeeded(&self) -> bool {
        *self
    }
}
macro_rules! always_succeeds {
    ($($t:ty),*) => {$(impl Outcome for $t { fn succeeded(&self) -> bool { true } })*};
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
    fn succeeded(&self) -> bool {
        self.0.succeeded() && self.1.succeeded()
    }
}
impl<T: Outcome> Outcome for Vec<T> {
    fn succeeded(&self) -> bool {
        self.iter().all(Outcome::succeeded)
    }
}

#[derive(Clone, Debug)]
struct Point {
    magnitude: f64,
    median_ns: f64,
    min_ns: f64,
    max_ns: f64,
    iterations: u64,
    status: &'static str,
}

#[derive(Clone, Debug)]
struct Series {
    kind: String,
    points: Vec<Point>,
}

#[derive(Clone, Debug)]
struct Grid {
    kind: String,
    a_label: String,
    b_label: String,
    a: Vec<f64>,
    b: Vec<f64>,
    ns: Vec<Vec<Option<f64>>>,
    errors: Vec<Vec<bool>>,
}

#[derive(Clone, Debug)]
struct Operation {
    name: String,
    group: String,
    description: String,
    magnitude: String,
    series: Vec<Series>,
    fuzz: Vec<Series>,
    grids: Vec<Grid>,
}

/// Collects sweeps for one crate and writes them as JSON.
pub struct Atlas {
    crate_name: String,
    config: Config,
    output: Option<String>,
    operations: Vec<Operation>,
    started: Instant,
}

enum Measured {
    Done(Point),
    TooSlow(f64),
}

impl Atlas {
    /// Parse `--out PATH`, `--quick`, `--filter TEXT` and `--seed N` from the
    /// process arguments. Without `--out`, the JSON goes to standard output.
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

    /// An atlas with explicit settings.
    pub fn new(crate_name: &str, config: Config, output: Option<String>) -> Self {
        Self {
            crate_name: crate_name.to_owned(),
            config,
            output,
            operations: Vec::new(),
            started: Instant::now(),
        }
    }

    /// Settings in effect.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// A generator for one operation/kind, so sweeps do not depend on order.
    pub fn rng(&self, operation: &str, kind: &str) -> Rng {
        let mut hash = self.config.seed;
        for byte in operation.bytes().chain([0]).chain(kind.bytes()) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
        }
        Rng::new(hash)
    }

    fn selected(&self, operation: &str) -> bool {
        self.config
            .filter
            .as_ref()
            .is_none_or(|filter| operation.contains(filter.as_str()))
    }

    /// Declare an operation. `magnitude` names the swept axis, for example
    /// "operand bits" or "polynomial degree".
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

    fn current(&mut self) -> &mut Operation {
        self.operations
            .last_mut()
            .expect("declare an operation before sweeping it")
    }

    fn measure(config: &Config, run: &mut dyn FnMut() -> bool) -> Measured {
        let start = Instant::now();
        let succeeded = run();
        let first = start.elapsed();
        let ok = if succeeded { "ok" } else { "error" };
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
            });
        }
        let mut iterations: u64 = 1;
        let mut batch = first;
        while batch < config.target_batch && start.elapsed() < config.point_budget / 2 {
            iterations = iterations.saturating_mul(2);
            let t = Instant::now();
            for _ in 0..iterations {
                black_box(run());
            }
            batch = t.elapsed();
        }
        let mut samples = vec![batch.as_nanos() as f64 / iterations as f64];
        while samples.len() < config.samples && start.elapsed() < config.point_budget {
            let t = Instant::now();
            for _ in 0..iterations {
                black_box(run());
            }
            samples.push(t.elapsed().as_nanos() as f64 / iterations as f64);
        }
        samples.sort_by(f64::total_cmp);
        Measured::Done(Point {
            magnitude: 0.0,
            median_ns: samples[samples.len() / 2],
            min_ns: samples[0],
            max_ns: samples[samples.len() - 1],
            iterations,
            status: ok,
        })
    }

    /// Sweep one input kind over `magnitudes`. `make` builds an input for a
    /// magnitude (outside the timer); `run` is the timed operation.
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
        for &magnitude in magnitudes {
            if stopped {
                points.push(Point {
                    magnitude,
                    median_ns: f64::NAN,
                    min_ns: f64::NAN,
                    max_ns: f64::NAN,
                    iterations: 0,
                    status: "skipped",
                });
                continue;
            }
            let input = make(magnitude, &mut rng);
            let point = match Self::measure(&self.config, &mut || {
                let result = run(black_box(&input));
                let succeeded = result.succeeded();
                black_box(result);
                succeeded
            }) {
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
                    }
                }
            };
            points.push(point);
        }
        self.current().series.push(Series {
            kind: kind.to_owned(),
            points,
        });
    }

    /// Time `fuzz_per_magnitude` random inputs at each magnitude, each with a
    /// single short calibrated batch, to show the spread hidden by a median.
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
        'magnitudes: for &magnitude in magnitudes {
            for _ in 0..self.config.fuzz_per_magnitude {
                let input = make(magnitude, &mut rng);
                match Self::measure(&config, &mut || {
                    let result = run(black_box(&input));
                    let succeeded = result.succeeded();
                    black_box(result);
                    succeeded
                }) {
                    Measured::Done(point) => points.push(Point { magnitude, ..point }),
                    Measured::TooSlow(ns) => {
                        points.push(Point {
                            magnitude,
                            median_ns: ns,
                            min_ns: ns,
                            max_ns: ns,
                            iterations: 1,
                            status: "ceiling",
                        });
                        break 'magnitudes;
                    }
                }
            }
        }
        self.current().fuzz.push(Series {
            kind: kind.to_owned(),
            points,
        });
    }

    /// Sweep a two-argument operation over a magnitude grid. Cells past a
    /// too-slow cell in the same row are skipped.
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
        for &y in b {
            let mut row = Vec::new();
            let mut failed = Vec::new();
            let mut stopped = false;
            for &x in a {
                if stopped {
                    row.push(None);
                    failed.push(false);
                    continue;
                }
                let input = make(x, y, &mut rng);
                match Self::measure(&config, &mut || {
                    let result = run(black_box(&input));
                    let succeeded = result.succeeded();
                    black_box(result);
                    succeeded
                }) {
                    Measured::Done(point) => {
                        row.push(Some(point.median_ns));
                        failed.push(point.status == "error");
                    }
                    Measured::TooSlow(ns) => {
                        stopped = true;
                        row.push(Some(ns));
                        failed.push(false);
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

    /// Write the JSON document to `--out`, or standard output.
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

fn number(value: f64) -> String {
    if value.is_finite() {
        format!("{value}")
    } else {
        "null".to_owned()
    }
}

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

fn series_json(series: &Series) -> String {
    let points: Vec<String> = series
        .points
        .iter()
        .map(|p| {
            format!(
                "[{},{},{},{},{},{}]",
                number(p.magnitude),
                number(p.median_ns),
                number(p.min_ns),
                number(p.max_ns),
                p.iterations,
                quote(p.status)
            )
        })
        .collect();
    format!(
        "{{\"kind\":{},\"points\":[{}]}}",
        quote(&series.kind),
        points.join(",")
    )
}

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

/// Powers of two from `2^lo` to `2^hi`, as magnitudes.
pub fn powers_of_two(lo: i32, hi: i32) -> Vec<f64> {
    (lo..=hi).map(|e| 2f64.powi(e)).collect()
}

/// A roughly geometric integer ladder from `lo` to `hi` with `steps` points.
pub fn ladder(lo: u64, hi: u64, steps: usize) -> Vec<f64> {
    let (lo, hi) = (lo.max(1) as f64, hi.max(1) as f64);
    let mut out: Vec<f64> = (0..steps.max(2))
        .map(|i| (lo * (hi / lo).powf(i as f64 / (steps.max(2) - 1) as f64)).round())
        .collect();
    out.dedup();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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
        assert_eq!(ladder(1, 1000, 4), vec![1.0, 10.0, 100.0, 1000.0]);
    }
}
