//! Performance atlas for Cartesian interval algebra across scalar backends.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperlattice-interval --all-features --example perf_atlas -- --out atlas.json`.
//! The shared runner accepts `--quick`, `--filter TEXT` and `--seed U64` as well.
//!
//! # Input and timing contracts
//!
//! Factories build owned fixtures outside callback timing. A callback creates fresh
//! backend contexts and performs its specified workload; some workloads include
//! cloning, validation or several dependent operations. Most callbacks black-box
//! and drop Results through done; Bisect short-circuits unmatched outcomes.
//! Preparation errors become constant-false jobs, and
//! runtime Result errors become false via done, losing their typed cause. The runner
//! classifies the initial callback outcome; later timing batches do not independently
//! validate every result. A true callback is arithmetic completion, not an independent
//! correctness check or necessarily a true inclusion/contraction certificate.
//!
//! The source Pair is a generator recipe, not a common exact operand for all backends.
//! Fixed nonpoints are rounded outward, but points floor to one lattice point. The
//! floating converter reduces to at most 62 integer bits, casts to binary64 and scales
//! in chunks; these latter operations are not directed rounding and do not certify
//! containment of the recipe's exact endpoints. Its point branch retains one float.
//! Dyadics preserve the unperturbed recipe exactly; rationals apply extra fractions.
//! Benchmark success is relative to these admitted backend inputs. The series label
//! "narrow (1 ulp)" denotes one source-mantissa step before perturbation/conversion,
//! not necessarily one representable step of the destination backend.
//!
//! Only dyadic callbacks have an arithmetic size/work budget, freshly initialized
//! per call; this does not bound factory allocation or whole-process work/memory.
//! The runner catches unwinding panics and checks timing ceilings after calls; it
//! cannot interrupt a hung call or prevent allocation failure. Axis settings and
//! finite seeded samples are a cost exploration, not a conformance or closure proof.
//!
//! # Workload catalog
//!
//! The generated marker types and their Op implementations select these factories:
//!
//! - `VectorAdd`: sum two independent vectors lane by lane.
//! - `VectorScale`: multiply each lane by one independently generated scalar interval.
//! - `Dot`: ordered products and sums of independent vector lanes in one context.
//! - `Hull`: Cartesian hull of two vectors.
//! - `Intersection`: intersect a with a precomputed hull containing a.
//! - `Subset`: test a against that containing hull; Ok(false) would still count as completion.
//! - `VectorNormOne`: enclose the sum of absolute lane values.
//! - `VectorNormInfinity`: enclose the largest absolute lane value.
//! - `Widths`: compute every lane width on the backend's width carrier.
//! - `WidestAxis`: compare widths and select the earliest widest lane.
//! - `VectorMidpointRadius`: retain the source outside timing, then query midpoint and
//!   radius with one context, skipping radius if midpoint fails.
//! - `CenteredVectorEnclosure`: clone center/radius, validate the authored form, then
//!   project it; cloning and validation occur inside the callback.
//! - `MatrixAdd`: entrywise sum of two square matrices.
//! - `MatrixVector`: a square matrix acting on independent input lanes.
//! - `MatrixMatrix`: ordered dot products for a square matrix product.
//! - `AffineEnclosure`: enclose A*x+t with independently generated interval entries.
//! - `Transpose`: owned transpose through backend copying/accounting.
//! - `MatrixNormOne`: enclose the maximum absolute column sum.
//! - `MatrixNormInfinity`: enclose the maximum absolute row sum.
//! - `MeasureInfinity`: diagonal entries plus off-diagonal absolute row sums, then maximum.
//! - `RowDominance`: absolute diagonal minus off-diagonal absolute row sum per row.
//! - `IdentityMinus`: form I-A without asserting that A is a particular remainder.
//! - `ContractionMargin`: compute 1 minus the infinity-norm enclosure; no positivity test.
//! - `MatrixMidpointRadius`: midpoint then radius queries, sharing context and short-circuiting.
//! - `CenteredMatrixEnclosure`: clone and validate point-center/nonnegative-point-radius
//!   matrices in the callback, then project their Cartesian enclosure.
//! - `AffineImageEnclosure`: project a retained point-map image built outside timing.
//! - `AffineImageRestrict`: restrict to a precomputed containing hull, then project
//!   with the same context. This exercises restriction without shrinking the source.
//! - `TranslatedBoxImage`: prepare an authored center/radius source and its point-map
//!   image outside timing, then project that retained definition.
//! - `Chain`: prepare up to 64 square point maps, then project their retained composition.
//!   Reducing map-entry exponents by two is not a contraction or overflow guarantee.
//! - `Krawczyk`: compute residual, correction, remainder, contraction margin and
//!   K=c-Y(Ac-b)+(I-YA)(X-c), then test interior inclusion. The job accepts Ok even
//!   when inclusion is false or the margin is not positive; no root theorem is asserted.
//!   Explicit point/narrow shape overrides mean the by-shape label does not control
//!   the Krawczyk box or coefficients. Signed point radii are hulled to construct X.
//! - `Bisect`: choose the widest axis during preparation, then require a successful
//!   Some pair of halves; explicit point fixtures can produce an unsuccessful job.
//! - `Regrid`: project outward to the grid 2^(spec.exp-16), without changing the source.

use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F64Enclosure, FixedInterval, IntervalError, RationalInterval,
};
use hyperlattice_interval::{
    AffineChain, AffineMap, CenteredMatrix, CenteredVector, IntervalArithmetic,
    IntervalLatticeError, IntervalMatrix, IntervalVector,
};
use num_bigint::{BigInt, Sign};
use perf_atlas::{Atlas, Rng, powers_of_two};
use std::hint::black_box;

/// Signed i32 fixed intervals with 16 fractional bits; generated points may be quantized.
type Q16 = FixedInterval<i32, 16>;
/// Signed i64 fixed intervals with 32 fractional bits for the wider fixed backend.
type Q32 = FixedInterval<i64, 32>;
/// Native Cartesian error used while preparing or executing jobs, before Boolean reduction.
type Error = IntervalLatticeError;

/// Exact series/dispatch label for Q16; used in magnitude sweeps.
const FIXED16: &str = "fixed i32 Q16";
/// Exact series/dispatch label for Q32; also used in dimension and range sweeps.
const FIXED32: &str = "fixed i64 Q32";
/// Exact series/dispatch label for the admitted F64Enclosure backend.
const FLOAT: &str = "f64 enclosure";
/// Exact series/dispatch label for arbitrary-mantissa dyadic intervals.
const DYADIC: &str = "dyadic";
/// Exact series/dispatch label for independently perturbed rational endpoints.
const RATIONAL: &str = "rational";
/// The four backends of the dimension sweeps.
const BACKENDS: [&str; 4] = [FIXED32, FLOAT, DYADIC, RATIONAL];
/// Every instantiated const-generic dimension.
const DIMS: [usize; 10] = [1, 2, 3, 4, 6, 8, 12, 16, 24, 32];

#[derive(Clone, Copy, Debug, PartialEq)]
/// Source-mantissa shape, before backend quantization or rational perturbation.
enum Shape {
    /// One mantissa value; rational perturbations also match, preserving singleton input.
    Point,
    /// Adjacent mantissas with width one at the recipe scale, before conversion.
    Narrow,
    /// Same-sign interval with an independently drawn positive width.
    Wide,
    /// Independent negative/positive mantissas; overrides the nonnegative sign request.
    Straddle,
}

/// Stable display labels and generator choices; their order also drives random selection.
const SHAPES: [(&str, Shape); 4] = [
    ("point", Shape::Point),
    ("narrow (1 ulp)", Shape::Narrow),
    ("wide", Shape::Wide),
    ("zero-straddling", Shape::Straddle),
];

/// How to draw interval entries. Endpoints are `m·2^scale` with `bits`-bit
/// mantissas `m` and magnitude about `2^exp`; positive `spread` selects each
/// entry's exponent from the inclusive range `exp ± spread` by remainder reduction.
/// `shape: None` draws a shape per entry. Rational endpoints add perturbations
/// over an odd denominator. Width is clamped to two; arbitrary settings are unvalidated.
#[derive(Clone, Copy, Debug)]
struct Spec {
    /// Requested source shape; None selects a shape per entry unless the caller overrides it.
    shape: Option<Shape>,
    /// Requested mantissa width, clamped to at least two by pair.
    bits: u32,
    /// Nominal leading-bit exponent before per-entry spread and rational perturbation.
    exp: i32,
    /// Positive values draw an inclusive exponent offset in ±spread; nonpositive values add zero.
    spread: i32,
    /// Chain length for affine chains.
    count: usize,
}

/// Defaults for a random-shape fixture recipe; this is not a validated configuration type.
impl Spec {
    /// Store requested width/exponent, random shapes, zero spread and one map.
    /// No range/resource checks are performed; callers keep arithmetic and shifts in range.
    fn new(bits: u32, exp: i32) -> Self {
        Self {
            shape: None,
            bits,
            exp,
            spread: 0,
            count: 1,
        }
    }
}

/// An exact endpoint pair `[lo, hi]·2^scale`, plus a rational perturbation:
/// rational backends use `(lo·den + r_lo) / den` and `(hi·den + r_hi) / den`,
/// each multiplied by `2^scale`. Other backends ignore the perturbations.
struct Pair {
    /// Lower unperturbed source mantissa, ordered with hi by pair.
    lo: BigInt,
    /// Upper unperturbed source mantissa, equal to lo for a point recipe.
    hi: BigInt,
    /// Shared power-of-two exponent applied to both unperturbed and rational endpoints.
    scale: i64,
    /// Positive odd denominator used only by the rational backend.
    den: BigInt,
    /// Positive rational lower-endpoint numerator perturbation, smaller than den.
    r_lo: BigInt,
    /// Positive rational upper perturbation; equals r_lo for points.
    r_hi: BigInt,
}

/// Draw a positive BigInt of max(bits,1) bits through the shared deterministic RNG.
/// Limb allocation scales with width and has no separate preparation budget.
fn big(bits: u32, rng: &mut Rng) -> BigInt {
    BigInt::from_slice(Sign::Plus, &rng.limbs(bits))
}

/// Generate ordered source mantissas and rational perturbations from the shared RNG.
/// An explicit shape overrides spec.shape; otherwise selection uses remainder reduction,
/// not an exact-uniform distribution guarantee. Width is clamped to two. Positive
/// spread draws an inclusive exponent offset. Nonnegative suppresses random sign
/// selection for point/narrow/wide, but Straddle still crosses zero. Even backends
/// that ignore rational perturbations consume their RNG draws. Within the configured
/// sweep ranges, positive sub-denominator perturbations preserve endpoint ordering;
/// there is no validation of arbitrary Spec overflow, huge shifts or allocation.
fn pair(spec: &Spec, shape: Option<Shape>, nonnegative: bool, rng: &mut Rng) -> Pair {
    let bits = spec.bits.max(2);
    let shape = shape
        .or(spec.shape)
        .unwrap_or_else(|| SHAPES[rng.below(4) as usize].1);
    let exp = spec.exp
        + if spec.spread > 0 {
            rng.range(-i64::from(spec.spread), i64::from(spec.spread)) as i32
        } else {
            0
        };
    let negative = !nonnegative && rng.coin();
    let one = BigInt::from(1);
    let (lo, hi) = match shape {
        Shape::Point => {
            let m = big(bits, rng);
            let m = if negative { -m } else { m };
            (m.clone(), m)
        }
        Shape::Narrow => {
            let m = big(bits, rng);
            if negative {
                (-&m - &one, -m)
            } else {
                (m.clone(), m + one)
            }
        }
        Shape::Wide => {
            let c = big(bits, rng);
            let w = big(bits - 1, rng);
            if negative {
                (-(&c + &w), -c)
            } else {
                (c.clone(), c + w)
            }
        }
        Shape::Straddle => (-big(bits, rng), big(bits, rng)),
    };
    let r_lo = big(bits - 1, rng);
    let r_hi = if shape == Shape::Point {
        r_lo.clone()
    } else {
        big(bits - 1, rng)
    };
    Pair {
        lo,
        hi,
        scale: i64::from(exp) - i64::from(bits) + 1,
        den: big(bits, rng) | BigInt::from(1),
        r_lo,
        r_hi,
    }
}

/// Compute `floor(m*2^shift)` or `ceil(m*2^shift)` exactly with BigInt shifts.
/// Negative right shifts round toward negative infinity; double negation produces
/// ceiling. Callers bound shifts to fit usize and avoid i64::MIN negation.
/// Allocation grows with a positive shift, without arithmetic-budget admission.
fn shifted(m: &BigInt, shift: i64, up: bool) -> BigInt {
    if shift >= 0 {
        m << shift as usize
    } else if up {
        -((-m) >> (-shift) as usize)
    } else {
        m >> (-shift) as usize
    }
}

/// Scale a float by powers of two in chunks of at most 1000 exponent units.
/// Intermediate underflow/overflow or rounding is not repaired; the number of loop
/// iterations grows with the exponent. This is fixture conversion, not directed rounding.
fn ldexp(mut x: f64, mut e: i64) -> f64 {
    while e > 1000 {
        x *= 2f64.powi(1000);
        e -= 1000;
    }
    while e < -1000 {
        x *= 2f64.powi(-1000);
        e += 1000;
    }
    x * 2f64.powi(e as i32)
}

/// Reduce to at most 62 integer bits with the requested floor/ceiling, cast to f64,
/// then apply the adjusted exponent. The final cast/scaling can round either way,
/// so up controls only integer truncation, not a certified real endpoint bound.
/// The expect relies on that bit reduction; configured recipes keep exponent arithmetic
/// and shifts representable. Extreme conversions can produce zero or infinity.
fn to_f64(m: &BigInt, scale: i64, up: bool) -> f64 {
    let extra = m.bits().saturating_sub(62) as i64;
    let top = i64::try_from(shifted(m, -extra, up)).expect("62-bit mantissa");
    ldexp(top as f64, scale + extra)
}

/// Atlas-only fixture conversion and fresh-context interface over native interval arithmetic.
/// Conversions may quantize the recipe; successful operations are exact/enclosing only
/// relative to the resulting backend carrier. This trait is not a lossless adapter contract.
trait Backend: IntervalArithmetic + 'static {
    /// Create a new context for one callback or explicit preparation operation.
    fn context() -> Self::Context;
    /// Admit one generated recipe under this backend's conversion rules; native failures propagate.
    fn from_pair(p: &Pair) -> Result<Self, IntervalError>;
}

/// Implement fixed-grid fixture conversion for the two selected storage/scale pairs.
/// The template is an atlas adapter, not the native exact scalar conversion authority.
macro_rules! fixed_backend {
    ($t:ty, $int:ty, $frac:expr) => {
        /// Quantize unperturbed mantissas to this fixed grid and discard rational perturbations.
        impl Backend for $t {
            /// Use the unit context; fixed arithmetic has no caller budget.
            fn context() {}
            /// Floor the scaled lower mantissa and narrow to endpoint storage. A point returns
            /// that single value; a nonpoint also ceilings/narrows the upper bound. Conversion
            /// errors are Overflow, and native constructors validate scale/order.
            fn from_pair(p: &Pair) -> Result<Self, IntervalError> {
                let shift = p.scale + $frac;
                let lo = <$int>::try_from(shifted(&p.lo, shift, false))
                    .map_err(|_| IntervalError::Overflow)?;
                // A point rounds to one lattice point, so it stays a point.
                if p.lo == p.hi {
                    return Self::point(lo);
                }
                let hi = <$int>::try_from(shifted(&p.hi, shift, true))
                    .map_err(|_| IntervalError::Overflow)?;
                Self::new(lo, hi)
            }
        }
    };
}
fixed_backend!(Q16, i32, 16);
fixed_backend!(Q32, i64, 32);

/// Construct native float carriers from approximate recipe endpoints; qualification
/// and finiteness are validated by F64Enclosure, not proved by this conversion.
impl Backend for F64Enclosure {
    /// Use the unit context; float qualification is checked by native operations.
    fn context() {}
    /// Convert unperturbed lower/upper recipes to floats and invoke native admission.
    /// Points reuse only the lower conversion. Infinity or an unsupported floating
    /// environment remains a native error; recipe containment is not certified.
    fn from_pair(p: &Pair) -> Result<Self, IntervalError> {
        let lo = to_f64(&p.lo, p.scale, false);
        if p.lo == p.hi {
            return Self::point(lo);
        }
        Self::new(lo, to_f64(&p.hi, p.scale, true))
    }
}

/// Preserve unperturbed exact dyadic endpoints and use an explicit fresh arithmetic budget.
impl Backend for DyadicInterval {
    /// Create a 2^24-bit size budget with u64::MAX work units. This is fresh per call,
    /// not a total atlas/factory resource bound.
    fn context() -> DyadicBudget {
        DyadicBudget::new(1 << 24, u64::MAX)
    }
    /// Check the shared exponent fits i32, clone mantissas and construct exact dyadic
    /// bounds. Native normalization/validation applies; preparation is outside this budget.
    fn from_pair(p: &Pair) -> Result<Self, IntervalError> {
        let scale = i32::try_from(p.scale).map_err(|_| IntervalError::Overflow)?;
        Self::new(p.lo.clone(), p.hi.clone(), scale)
    }
}

/// Apply rational perturbations before exact power-of-two scaling and native admission.
impl Backend for RationalInterval {
    /// Use the unit context; this atlas imposes no rational arithmetic budget.
    fn context() {}
    /// Form perturbed integer numerators and shift numerators or denominator exactly.
    /// Parse generated numerator/positive-denominator strings, then validate endpoint
    /// order. Parse expects rely on generated integer syntax; allocations are unbudgeted.
    fn from_pair(p: &Pair) -> Result<Self, IntervalError> {
        let mut lo = &p.lo * &p.den + &p.r_lo;
        let mut hi = &p.hi * &p.den + &p.r_hi;
        let mut den = p.den.clone();
        if p.scale >= 0 {
            lo <<= p.scale as usize;
            hi <<= p.scale as usize;
        } else {
            den <<= (-p.scale) as usize;
        }
        Self::new(
            format!("{lo}/{den}").parse().expect("generated rational"),
            format!("{hi}/{den}").parse().expect("generated rational"),
        )
    }
}

/// Generate a recipe, convert it, and wrap any native scalar error as the atlas Error.
/// This advances the RNG even when conversion later fails.
fn scalar<B: Backend>(
    spec: &Spec,
    shape: Option<Shape>,
    nonnegative: bool,
    rng: &mut Rng,
) -> Result<B, Error> {
    Ok(B::from_pair(&pair(spec, shape, nonnegative, rng))?)
}

/// Build N values in order via a temporary Vec, stopping and dropping the prefix
/// on the first error. Successful length is exactly N, justifying conversion to the
/// array; N=0 invokes no callback. This allocating helper runs in fixture preparation.
fn array<T, const N: usize>(mut f: impl FnMut() -> Result<T, Error>) -> Result<[T; N], Error> {
    let items = (0..N).map(|_| f()).collect::<Result<Vec<T>, Error>>()?;
    Ok(items.try_into().unwrap_or_else(|_| unreachable!("N items")))
}

/// Prepare independently generated lanes with one optional shape override and validate
/// the Cartesian vector. The first scalar/constructor error aborts preparation.
fn vector<B: Backend, const N: usize>(
    spec: &Spec,
    shape: Option<Shape>,
    rng: &mut Rng,
) -> Result<IntervalVector<B, N>, Error> {
    IntervalVector::new(array(|| scalar(spec, shape, false, rng))?)
}

/// Prepare point radii with nonnegative sign selection and native vector validation.
/// Explicit point shape prevents a random Straddle from overriding nonnegativity.
fn radius_vector<B: Backend, const N: usize>(
    spec: &Spec,
    rng: &mut Rng,
) -> Result<IntervalVector<B, N>, Error> {
    IntervalVector::new(array(|| scalar(spec, Some(Shape::Point), true, rng))?)
}

/// Prepare independent entries in row-major order, using nested temporary vectors,
/// then validate the Cartesian matrix; errors stop before a job is returned.
fn matrix<B: Backend, const R: usize, const C: usize>(
    spec: &Spec,
    shape: Option<Shape>,
    rng: &mut Rng,
) -> Result<IntervalMatrix<B, R, C>, Error> {
    IntervalMatrix::new(array(|| array(|| scalar(spec, shape, false, rng)))?)
}

/// Prepare a square singleton map `x -> A*x+b` with the requested nominal exponent.
/// Shape is explicitly Point; the recipe's remaining fields, including spread, are
/// retained. Build matrix before translation and propagate map validation failure.
fn point_map<B: Backend, const N: usize>(
    spec: &Spec,
    exp: i32,
    rng: &mut Rng,
) -> Result<AffineMap<B, N, N>, Error> {
    let spec = Spec { exp, ..*spec };
    AffineMap::new(
        matrix(&spec, Some(Shape::Point), rng)?,
        vector(&spec, Some(Shape::Point), rng)?,
    )
}

/// Owned repeatable callback with captured fixtures; true denotes its completion test.
/// Box allocation occurs during preparation. Output values and typed errors are not retained.
type Job = Box<dyn Fn() -> bool>;

/// Observe Result success, black-box and drop the entire result, then return that bit.
/// Ok(false), Ok(None) and nonpositive certificate margins still count as success.
/// Error identity and successful output evidence are intentionally lost in this atlas.
fn done<T>(result: Result<T, Error>) -> bool {
    let ok = result.is_ok();
    let _ = black_box(result);
    ok
}

/// An operation instantiated per backend and dimension.
trait Op {
    /// Prepare a repeatable owned job for this backend/dimension outside callback timing.
    /// Return preparation failures before execution; dispatch converts them to false jobs.
    fn build<B: Backend, const N: usize>(&self, spec: &Spec, rng: &mut Rng) -> Result<Job, Error>;
}

/// Generate a workload marker and a non-inlined generic factory for each catalog entry.
/// Keeping build in separate frames limits dispatcher stack accumulation; it is not
/// a uniform stack/memory bound for all instantiations.
macro_rules! op {
    ($name:ident, |$spec:ident, $rng:ident| $body:block) => {
        /// Prepared-workload selector implementing Op. Its mathematical operation and timed
        /// boundary are specified by name in the [workload catalog](index.html#workload-catalog).
        struct $name;
        /// Bind this catalog workload to the shared backend/dimension factory interface.
        impl Op for $name {
            // Separate frames keep large instantiations off the dispatcher's stack.
            #[inline(never)]
            /// Build the catalog workload's owned inputs, then return its repeatable callback.
            /// This non-inlined factory can allocate and fail before timing; its particular
            /// operation, context sharing and output-completion rule are in the workload catalog.
            fn build<B: Backend, const N: usize>(
                &self,
                $spec: &Spec,
                $rng: &mut Rng,
            ) -> Result<Job, Error> {
                $body
            }
        }
    };
}

/// Dispatch the exact backend label and one instantiated dimension to its factory.
/// Unknown labels/dimensions panic as caller invariant failures. A preparation Err
/// becomes a boxed constant-false callback, discarding its cause and failed setup cost.
fn job<O: Op>(op: &O, backend: &str, n: usize, spec: &Spec, rng: &mut Rng) -> Job {
    /// Instantiate only DIMS for the selected backend; an unsupported dimension panics.
    macro_rules! dims {
        ($b:ty) => {
            match n {
                1 => op.build::<$b, 1>(spec, rng),
                2 => op.build::<$b, 2>(spec, rng),
                3 => op.build::<$b, 3>(spec, rng),
                4 => op.build::<$b, 4>(spec, rng),
                6 => op.build::<$b, 6>(spec, rng),
                8 => op.build::<$b, 8>(spec, rng),
                12 => op.build::<$b, 12>(spec, rng),
                16 => op.build::<$b, 16>(spec, rng),
                24 => op.build::<$b, 24>(spec, rng),
                32 => op.build::<$b, 32>(spec, rng),
                other => unreachable!("dimension {other} is not instantiated"),
            }
        };
    }
    let built = match backend {
        FIXED16 => dims!(Q16),
        FIXED32 => dims!(Q32),
        FLOAT => dims!(F64Enclosure),
        DYADIC => dims!(DyadicInterval),
        RATIONAL => dims!(RationalInterval),
        other => unreachable!("unknown backend {other}"),
    };
    // An input the backend cannot represent is an immediately failed call.
    built.unwrap_or_else(|_| Box::new(|| false))
}

/// Invoke the prepared Boolean callback without checking or preserving its geometry.
fn run(job: &Job) -> bool {
    job()
}

/// Owned sweep axes represented as f64 for the atlas plotting interface.
/// Factories cast them back to dimensions/widths/exponents; the hardcoded caller
/// supplies exactly representable integers, not arbitrary validated user axes.
struct Settings {
    /// All ten compiled dimensions for sweeps and randomized timing samples.
    dims: Vec<f64>,
    /// Dimension subset for two-axis grids.
    grid_dims: Vec<f64>,
    /// Requested mantissa widths for shape series, including one which pair clamps to two.
    bits: Vec<f64>,
    /// Requested widths for dimension/bit grids and selected randomized samples.
    grid_bits: Vec<f64>,
    /// Sorted, deduplicated nominal magnitudes for overflow/conversion exploration.
    exponents: Vec<f64>,
    /// Nonnegative dynamic-range offsets around a zero nominal exponent.
    spreads: Vec<f64>,
}

/// Default sweeps: backends by dimension, 32-bit mantissas near 1 with random
/// shapes; rational and dyadic grids of dimension × endpoint bits; fuzzed
/// shapes, sizes and magnitudes per dimension.
fn by_dimension<O: Op>(
    atlas: &mut Atlas,
    s: &Settings,
    group: &str,
    name: &str,
    description: &str,
    op: &O,
) {
    atlas.operation(group, name, "dimension N", description);
    for backend in BACKENDS {
        atlas.sweep(
            backend,
            &s.dims,
            |n, rng| job(op, backend, n as usize, &Spec::new(32, 0), rng),
            run,
        );
    }
    for backend in [RATIONAL, DYADIC] {
        atlas.grid(
            backend,
            ("dimension N", &s.grid_dims),
            ("endpoint bits", &s.grid_bits),
            |n, bits, rng| job(op, backend, n as usize, &Spec::new(bits as u32, 0), rng),
            run,
        );
    }
    for backend in [RATIONAL, DYADIC, FLOAT] {
        atlas.fuzz(
            backend,
            &s.dims,
            |n, rng| {
                let spec = Spec::new(rng.range(1, 256) as u32, rng.range(-16, 16) as i32);
                job(op, backend, n as usize, &spec, rng)
            },
            run,
        );
    }
}

/// Register rational shape series at N=4 and randomized dyadic shapes over bit sizes.
/// An operation's explicit shape override can supersede a series request; labels
/// alone do not establish that its actual operands vary by the requested shape.
fn by_shape<O: Op>(
    atlas: &mut Atlas,
    s: &Settings,
    group: &str,
    name: &str,
    description: &str,
    op: &O,
) {
    atlas.operation(group, &format!("{name} by shape"), "endpoint bits", &format!("{description} Rational endpoints with odd denominators, dimension 4, one interval shape per series."));
    for (kind, shape) in SHAPES {
        atlas.sweep(
            kind,
            &s.bits,
            |bits, rng| {
                let spec = Spec {
                    shape: Some(shape),
                    ..Spec::new(bits as u32, 0)
                };
                job(op, RATIONAL, 4, &spec, rng)
            },
            run,
        );
    }
    atlas.fuzz(
        "dyadic, random shapes",
        &s.grid_bits,
        |bits, rng| {
            job(
                op,
                DYADIC,
                4,
                &Spec::new(rng.range(1, bits as i64) as u32, 0),
                rng,
            )
        },
        run,
    );
}

/// Register magnitude sweeps at N=4 and 24 requested mantissa bits for both fixed
/// families, float and dyadic. Conversion failure and operation failure share the
/// same Boolean outcome; this cannot by itself locate an arithmetic overflow boundary.
fn by_magnitude<O: Op>(
    atlas: &mut Atlas,
    s: &Settings,
    group: &str,
    name: &str,
    description: &str,
    op: &O,
) {
    atlas.operation(group, &format!("{name} by magnitude"), "magnitude exponent e (entries ≈ 2^e)", &format!("{description} 24-bit mantissas scaled to about 2^e, dimension 4, random shapes; overflow and unrepresentable inputs fail."));
    for backend in [FIXED16, FIXED32, FLOAT, DYADIC] {
        atlas.sweep(
            backend,
            &s.exponents,
            |e, rng| job(op, backend, 4, &Spec::new(24, e as i32), rng),
            run,
        );
    }
}

/// Dynamic range: entries with exponents drawn from ±R, allowing larger dyadic
/// alignment shifts and rational denominators as R grows. Individual samples
/// need not show monotone cost or size, and fixed conversion may quantize or fail.
fn by_range<O: Op>(
    atlas: &mut Atlas,
    s: &Settings,
    group: &str,
    name: &str,
    description: &str,
    op: &O,
) {
    atlas.operation(
        group,
        &format!("{name} by dynamic range"),
        "exponent spread R (entries ≈ 2^[-R, R])",
        &format!(
            "{description} 24-bit mantissas whose exponents are drawn from [-R, R], dimension 8."
        ),
    );
    for backend in BACKENDS {
        atlas.sweep(
            backend,
            &s.spreads,
            |r, rng| {
                let spec = Spec {
                    spread: r as i32,
                    ..Spec::new(24, 0)
                };
                job(op, backend, 8, &spec, rng)
            },
            run,
        );
    }
}

op!(VectorAdd, |spec, rng| {
    let (a, b) = (
        vector::<B, N>(spec, None, rng)?,
        vector::<B, N>(spec, None, rng)?,
    );
    Ok(Box::new(move || done(a.add(&b, &mut B::context()))))
});
op!(VectorScale, |spec, rng| {
    let a = vector::<B, N>(spec, None, rng)?;
    let s = scalar::<B>(spec, None, false, rng)?;
    Ok(Box::new(move || done(a.scale(&s, &mut B::context()))))
});
op!(Dot, |spec, rng| {
    let (a, b) = (
        vector::<B, N>(spec, None, rng)?,
        vector::<B, N>(spec, None, rng)?,
    );
    Ok(Box::new(move || done(a.dot(&b, &mut B::context()))))
});
op!(Hull, |spec, rng| {
    let (a, b) = (
        vector::<B, N>(spec, None, rng)?,
        vector::<B, N>(spec, None, rng)?,
    );
    Ok(Box::new(move || done(a.hull(&b, &mut B::context()))))
});
op!(Intersection, |spec, rng| {
    let (a, b) = (
        vector::<B, N>(spec, None, rng)?,
        vector::<B, N>(spec, None, rng)?,
    );
    let c = a.hull(&b, &mut B::context())?;
    Ok(Box::new(move || {
        done(a.intersection(&c, &mut B::context()))
    }))
});
op!(Subset, |spec, rng| {
    let (a, b) = (
        vector::<B, N>(spec, None, rng)?,
        vector::<B, N>(spec, None, rng)?,
    );
    let c = a.hull(&b, &mut B::context())?;
    Ok(Box::new(move || done(a.subset_of(&c, &mut B::context()))))
});
op!(VectorNormOne, |spec, rng| {
    let a = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.norm_one(&mut B::context()))))
});
op!(VectorNormInfinity, |spec, rng| {
    let a = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.norm_infinity(&mut B::context()))))
});
op!(Widths, |spec, rng| {
    let a = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.widths(&mut B::context()))))
});
op!(WidestAxis, |spec, rng| {
    let a = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.widest_axis(&mut B::context()))))
});
op!(VectorMidpointRadius, |spec, rng| {
    let form = vector::<B, N>(spec, None, rng)?.midpoint_radius();
    Ok(Box::new(move || {
        let mut context = B::context();
        done(form.midpoint_enclosure(&mut context)) && done(form.radius_enclosure(&mut context))
    }))
});
op!(CenteredVectorEnclosure, |spec, rng| {
    let center = vector::<B, N>(spec, Some(Shape::Point), rng)?;
    let radius = radius_vector::<B, N>(spec, rng)?;
    Ok(Box::new(move || {
        let centered = CenteredVector::new(center.clone(), radius.clone());
        done(centered.and_then(|c| c.enclosure(&mut B::context())))
    }))
});
op!(MatrixAdd, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    let b = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.add(&b, &mut B::context()))))
});
op!(MatrixVector, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    let x = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.mul_vector(&x, &mut B::context()))))
});
op!(MatrixMatrix, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    let b = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.mul_matrix(&b, &mut B::context()))))
});
op!(AffineEnclosure, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    let x = vector::<B, N>(spec, None, rng)?;
    let t = vector::<B, N>(spec, None, rng)?;
    Ok(Box::new(move || {
        done(a.affine_enclosure(&x, &t, &mut B::context()))
    }))
});
op!(Transpose, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.transpose(&mut B::context()))))
});
op!(MatrixNormOne, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.norm_one(&mut B::context()))))
});
op!(MatrixNormInfinity, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.norm_infinity(&mut B::context()))))
});
op!(MeasureInfinity, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || {
        done(a.measure_infinity(&mut B::context()))
    }))
});
op!(RowDominance, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || {
        done(a.row_dominance_margins(&mut B::context()))
    }))
});
op!(IdentityMinus, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || done(a.identity_minus(&mut B::context()))))
});
op!(ContractionMargin, |spec, rng| {
    let a = matrix::<B, N, N>(spec, None, rng)?;
    Ok(Box::new(move || {
        done(a.contraction_margin_infinity(&mut B::context()))
    }))
});
op!(MatrixMidpointRadius, |spec, rng| {
    let form = matrix::<B, N, N>(spec, None, rng)?.midpoint_radius();
    Ok(Box::new(move || {
        let mut context = B::context();
        done(form.midpoint_enclosure(&mut context)) && done(form.radius_enclosure(&mut context))
    }))
});
op!(CenteredMatrixEnclosure, |spec, rng| {
    let center = matrix::<B, N, N>(spec, Some(Shape::Point), rng)?;
    let radius = IntervalMatrix::<B, N, N>::new(array(|| {
        array(|| scalar(spec, Some(Shape::Point), true, rng))
    })?)?;
    Ok(Box::new(move || {
        let centered = CenteredMatrix::new(center.clone(), radius.clone());
        done(centered.and_then(|c| c.enclosure(&mut B::context())))
    }))
});
op!(AffineImageEnclosure, |spec, rng| {
    let map = point_map::<B, N>(spec, spec.exp, rng)?;
    let image = map.image(vector::<B, N>(spec, None, rng)?);
    Ok(Box::new(move || done(image.enclosure(&mut B::context()))))
});
op!(AffineImageRestrict, |spec, rng| {
    let map = point_map::<B, N>(spec, spec.exp, rng)?;
    let source = vector::<B, N>(spec, None, rng)?;
    let restriction = source.hull(&vector::<B, N>(spec, None, rng)?, &mut B::context())?;
    let image = map.image(source);
    Ok(Box::new(move || {
        let mut context = B::context();
        done(
            image
                .restrict_parameters(&restriction, &mut context)
                .and_then(|r| r.enclosure(&mut context)),
        )
    }))
});
op!(TranslatedBoxImage, |spec, rng| {
    let map = point_map::<B, N>(spec, spec.exp, rng)?;
    let centered = CenteredVector::new(
        vector::<B, N>(spec, Some(Shape::Point), rng)?,
        radius_vector::<B, N>(spec, rng)?,
    )?;
    let image = map.image(centered.affine_source(&mut B::context())?);
    Ok(Box::new(move || done(image.enclosure(&mut B::context()))))
});
op!(Chain, |spec, rng| {
    // Map entries near 1/4 keep row sums near 1 at small dimensions.
    let mut chain =
        AffineChain::<B, IntervalVector<B, N>, N, 64>::new(vector::<B, N>(spec, None, rng)?);
    for _ in 0..spec.count {
        chain.push(point_map::<B, N>(spec, spec.exp - 2, rng)?)?;
    }
    Ok(Box::new(move || done(chain.enclosure(&mut B::context()))))
});
op!(Krawczyk, |spec, rng| {
    // Linear system A x = b with A = I + E, |E| small and slightly uncertain,
    // a point preconditioner Y near I, a point center c and a box X around c.
    let small = Spec {
        exp: spec.exp - 4 - (usize::BITS - N.leading_zeros()) as i32,
        ..*spec
    };
    let unit = Spec {
        shape: Some(Shape::Point),
        bits: 1,
        exp: 0,
        ..*spec
    };
    let identity =
        IntervalMatrix::<B, N, N>::new(array(|| array(|| B::zero().map_err(Error::from)))?)?
            .identity_minus(&mut B::context())?;
    let a = identity.add(
        &matrix::<B, N, N>(&small, Some(Shape::Narrow), rng)?,
        &mut B::context(),
    )?;
    let y = identity.sub(
        &matrix::<B, N, N>(&small, Some(Shape::Point), rng)?,
        &mut B::context(),
    )?;
    let b = vector::<B, N>(spec, Some(Shape::Point), rng)?;
    let c = vector::<B, N>(spec, Some(Shape::Point), rng)?;
    let radius = vector::<B, N>(&unit, Some(Shape::Point), rng)?;
    let x = c
        .sub(&radius, &mut B::context())?
        .hull(&c.add(&radius, &mut B::context())?, &mut B::context())?;
    Ok(Box::new(move || {
        let ctx = &mut B::context();
        let step = (|| {
            // K(X) = c - Y (A c - b) + (I - Y A)(X - c), accepted if K ⊂ int X.
            let residual = a.mul_vector(&c, ctx)?.sub(&b, ctx)?;
            let correction = y.mul_vector(&residual, ctx)?;
            let remainder = y.mul_matrix(&a, ctx)?.identity_minus(ctx)?;
            let margin = remainder.contraction_margin_infinity(ctx)?;
            let spread = remainder.mul_vector(&x.sub(&c, ctx)?, ctx)?;
            let k = c.sub(&correction, ctx)?.add(&spread, ctx)?;
            Ok::<_, Error>((k.interior_subset_of(&x, ctx)?, margin))
        })();
        done(step)
    }))
});

/// Dyadic-only partition helpers, by dimension.
trait DyadicOp {
    /// Prepare one dimension-specific dyadic job, reporting construction/selection errors.
    fn build<const N: usize>(&self, spec: &Spec, rng: &mut Rng) -> Result<Job, Error>;
}

/// Dispatch a dyadic operation over DIMS; unknown dimensions panic and preparation
/// errors become constant-false jobs with no retained typed cause.
fn dyadic_job<O: DyadicOp>(op: &O, n: usize, spec: &Spec, rng: &mut Rng) -> Job {
    let built = match n {
        1 => op.build::<1>(spec, rng),
        2 => op.build::<2>(spec, rng),
        3 => op.build::<3>(spec, rng),
        4 => op.build::<4>(spec, rng),
        6 => op.build::<6>(spec, rng),
        8 => op.build::<8>(spec, rng),
        12 => op.build::<12>(spec, rng),
        16 => op.build::<16>(spec, rng),
        24 => op.build::<24>(spec, rng),
        32 => op.build::<32>(spec, rng),
        other => unreachable!("dimension {other} is not instantiated"),
    };
    built.unwrap_or_else(|_| Box::new(|| false))
}

/// Select a widest-axis exact dyadic split; callback success requires two returned halves.
struct Bisect;
/// Prepare the source and axis before timing, leaving only bisection in the callback.
impl DyadicOp for Bisect {
    /// Choose an explicit shape or one of the three nonpoint shapes, construct the
    /// box and select its widest axis outside timing. The callback uses a fresh budget;
    /// Ok(None) is false, unlike the general done helper. Explicit point requests
    /// remain allowed and can therefore produce a failed completion bit.
    fn build<const N: usize>(&self, spec: &Spec, rng: &mut Rng) -> Result<Job, Error> {
        // Random shapes exclude points, which have no axis to split.
        let shape = spec
            .shape
            .unwrap_or_else(|| SHAPES[1 + rng.below(3) as usize].1);
        let a = vector::<DyadicInterval, N>(spec, Some(shape), rng)?;
        let axis = a.widest_axis(&mut DyadicInterval::context())?.unwrap_or(0);
        Ok(Box::new(move || {
            let halves = a.bisect_axis(axis, &mut DyadicInterval::context());
            // A singleton axis cannot be split: not a successful bisection.
            matches!(halves, Ok(Some(_))) && done(halves)
        }))
    }
}

/// Select outward dyadic regridding at a fixed offset from the recipe exponent.
struct Regrid;
/// Prepare source bounds once and regrid with a fresh dyadic context on every call.
impl DyadicOp for Regrid {
    /// Construct the source and retain precision=spec.exp-16 outside timing. Each
    /// callback projects outward on that grid with a fresh budget; its output is dropped.
    /// Per-entry exponent spread means this is not always 16 bits below every leading bit.
    fn build<const N: usize>(&self, spec: &Spec, rng: &mut Rng) -> Result<Job, Error> {
        let a = vector::<DyadicInterval, N>(spec, None, rng)?;
        // Keep 16 bits below the leading bit.
        let precision = spec.exp - 16;
        Ok(Box::new(move || {
            done(a.regrid(precision, &mut DyadicInterval::context()))
        }))
    }
}

/// Register nonpoint shape series at N=4, a dimension/bit grid, and seeded randomized
/// samples. Explicit nonpoint shape applies only to series; other requests delegate
/// shape selection to the operation. Runner filtering and stop rules can omit points.
fn dyadic_operation<O: DyadicOp>(
    atlas: &mut Atlas,
    s: &Settings,
    name: &str,
    description: &str,
    op: &O,
) {
    atlas.operation("subdivision", name, "endpoint bits", description);
    for (kind, shape) in SHAPES.into_iter().skip(1) {
        atlas.sweep(
            kind,
            &s.bits,
            |bits, rng| {
                let spec = Spec {
                    shape: Some(shape),
                    ..Spec::new(bits as u32, 0)
                };
                dyadic_job(op, 4, &spec, rng)
            },
            run,
        );
    }
    atlas.grid(
        "random shapes",
        ("dimension N", &s.grid_dims),
        ("endpoint bits", &s.grid_bits),
        |n, bits, rng| dyadic_job(op, n as usize, &Spec::new(bits as u32, 0), rng),
        run,
    );
    atlas.fuzz(
        "random shapes",
        &s.dims,
        |n, rng| {
            let spec = Spec::new(rng.range(2, 512) as u32, rng.range(-64, 64) as i32);
            dyadic_job(op, n as usize, &spec, rng)
        },
        run,
    );
}

/// Parse runner options, register all workloads and sweep axes, then emit the JSON atlas.
/// Dimensions range from one to 32; shape widths reach 4096 bits, grid widths 1024,
/// and magnitude/spread axes reach 1024. Chain sweeps reach the stored capacity 64.
/// Registration order fixes RNG namespaces/output order; filters may leave declarations
/// without measured series. Runner output is stdout or the requested path; invalid
/// arguments and output I/O errors panic. No process-wide timeout is installed.
fn main() {
    let mut atlas = Atlas::from_args("hyperlattice-interval");
    let mut exponents: Vec<f64> = (-1024..=1024)
        .step_by(64)
        .chain((-48..=48).step_by(8))
        .map(f64::from)
        .collect();
    exponents.sort_by(f64::total_cmp);
    exponents.dedup();
    let s = Settings {
        dims: DIMS.iter().map(|&n| n as f64).collect(),
        grid_dims: vec![1.0, 2.0, 4.0, 8.0, 16.0, 32.0],
        bits: powers_of_two(1, 12),
        grid_bits: powers_of_two(1, 10),
        exponents,
        spreads: [0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]
            .map(f64::from)
            .to_vec(),
    };
    let s = &s;
    let a = &mut atlas;

    by_dimension(
        a,
        s,
        "vector",
        "vector add",
        "Lanewise interval sum a + b.",
        &VectorAdd,
    );
    by_shape(
        a,
        s,
        "vector",
        "vector add",
        "Lanewise interval sum.",
        &VectorAdd,
    );
    by_dimension(
        a,
        s,
        "vector",
        "vector scale",
        "Every lane times one interval scalar.",
        &VectorScale,
    );
    by_dimension(
        a,
        s,
        "vector",
        "dot",
        "Interval dot product: N products and N sums in one context.",
        &Dot,
    );
    by_shape(a, s, "vector", "dot", "Interval dot product.", &Dot);
    by_magnitude(a, s, "vector", "dot", "Interval dot product.", &Dot);
    by_range(a, s, "vector", "dot", "Interval dot product.", &Dot);
    by_dimension(
        a,
        s,
        "vector",
        "hull",
        "Lanewise smallest enclosing box of two boxes.",
        &Hull,
    );
    by_dimension(
        a,
        s,
        "vector",
        "intersection",
        "Lanewise intersection of a box with a box containing it.",
        &Intersection,
    );
    by_dimension(
        a,
        s,
        "vector",
        "subset of",
        "Inclusion test of a box in a box containing it (every lane checked).",
        &Subset,
    );
    by_dimension(
        a,
        s,
        "norms",
        "vector norm one",
        "Enclosure of the sum of lane absolute values.",
        &VectorNormOne,
    );
    by_dimension(
        a,
        s,
        "norms",
        "vector norm infinity",
        "Enclosure of the largest lane absolute value.",
        &VectorNormInfinity,
    );
    by_dimension(
        a,
        s,
        "widths",
        "widths",
        "Exact width of every lane.",
        &Widths,
    );
    by_shape(a, s, "widths", "widths", "Exact lane widths.", &Widths);
    by_dimension(
        a,
        s,
        "widths",
        "widest axis",
        "Index of the widest lane by exact width comparison.",
        &WidestAxis,
    );
    by_dimension(
        a,
        s,
        "center-radius",
        "midpoint-radius vector",
        "Outward midpoint and radius enclosures of every lane.",
        &VectorMidpointRadius,
    );
    by_shape(
        a,
        s,
        "center-radius",
        "midpoint-radius vector",
        "Outward midpoint and radius enclosures.",
        &VectorMidpointRadius,
    );
    by_dimension(
        a,
        s,
        "center-radius",
        "centered vector enclosure",
        "Validate an exact center/radius box (point center, nonnegative point radius) and project it to its Cartesian enclosure.",
        &CenteredVectorEnclosure,
    );

    by_dimension(
        a,
        s,
        "matrix",
        "matrix add",
        "Entrywise interval sum of two N×N matrices.",
        &MatrixAdd,
    );
    by_dimension(
        a,
        s,
        "matrix",
        "matrix × vector",
        "N×N interval matrix times an interval vector.",
        &MatrixVector,
    );
    by_shape(
        a,
        s,
        "matrix",
        "matrix × vector",
        "Interval matrix-vector product.",
        &MatrixVector,
    );
    by_magnitude(
        a,
        s,
        "matrix",
        "matrix × vector",
        "Interval matrix-vector product.",
        &MatrixVector,
    );
    by_range(
        a,
        s,
        "matrix",
        "matrix × vector",
        "Interval matrix-vector product.",
        &MatrixVector,
    );
    by_dimension(
        a,
        s,
        "matrix",
        "matrix multiply",
        "N×N interval matrix product (N³ scalar products).",
        &MatrixMatrix,
    );
    by_shape(
        a,
        s,
        "matrix",
        "matrix multiply",
        "Interval matrix product.",
        &MatrixMatrix,
    );
    by_magnitude(
        a,
        s,
        "matrix",
        "matrix multiply",
        "Interval matrix product.",
        &MatrixMatrix,
    );
    by_dimension(
        a,
        s,
        "matrix",
        "affine enclosure",
        "Cartesian enclosure A·x + t of an interval affine expression.",
        &AffineEnclosure,
    );
    by_dimension(
        a,
        s,
        "matrix",
        "transpose",
        "Owned transpose through the arithmetic context.",
        &Transpose,
    );
    by_dimension(
        a,
        s,
        "norms",
        "matrix norm one",
        "Enclosure of the induced 1-norm (largest absolute column sum).",
        &MatrixNormOne,
    );
    by_dimension(
        a,
        s,
        "norms",
        "matrix norm infinity",
        "Enclosure of the induced ∞-norm (largest absolute row sum).",
        &MatrixNormInfinity,
    );
    by_shape(
        a,
        s,
        "norms",
        "matrix norm infinity",
        "Induced ∞-norm enclosure.",
        &MatrixNormInfinity,
    );
    by_dimension(
        a,
        s,
        "norms",
        "measure infinity",
        "Enclosure of the logarithmic ∞-norm (diagonal plus off-diagonal absolute row sums).",
        &MeasureInfinity,
    );
    by_dimension(
        a,
        s,
        "norms",
        "row dominance margins",
        "Per-row |a_ii| minus off-diagonal absolute sums.",
        &RowDominance,
    );
    by_dimension(
        a,
        s,
        "contraction",
        "identity minus",
        "I - A, the Krawczyk remainder matrix.",
        &IdentityMinus,
    );
    by_dimension(
        a,
        s,
        "contraction",
        "contraction margin infinity",
        "1 - ‖A‖∞, a contraction certificate margin.",
        &ContractionMargin,
    );
    by_dimension(
        a,
        s,
        "center-radius",
        "midpoint-radius matrix",
        "Outward midpoint and radius enclosures of every entry.",
        &MatrixMidpointRadius,
    );
    by_dimension(
        a,
        s,
        "center-radius",
        "centered matrix enclosure",
        "Validate an exact center/radius matrix and project it to its Cartesian enclosure.",
        &CenteredMatrixEnclosure,
    );

    by_dimension(
        a,
        s,
        "affine",
        "affine image enclosure",
        "Cartesian enclosure of the exact image of a box under a point map x -> A x + b.",
        &AffineImageEnclosure,
    );
    by_shape(
        a,
        s,
        "affine",
        "affine image enclosure",
        "Exact affine image enclosure.",
        &AffineImageEnclosure,
    );
    by_dimension(
        a,
        s,
        "affine",
        "affine image restrict",
        "Restrict an affine image's parameter box, then enclose the image.",
        &AffineImageRestrict,
    );
    by_dimension(
        a,
        s,
        "affine",
        "translated-box image enclosure",
        "Enclosure of the image of a center/offset source under a point map.",
        &TranslatedBoxImage,
    );

    a.operation("affine", "affine chain enclosure", "chain length (maps)", "Enclosure of a box pushed through k retained point maps (entries near 1/4), dimension 3; the chain composes maps before evaluating the source.");
    let lengths = [
        1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0,
    ];
    let chain = |backend: &str, n: usize, k: f64, rng: &mut Rng| {
        let spec = Spec {
            count: k as usize,
            ..Spec::new(32, 0)
        };
        job(&Chain, backend, n, &spec, rng)
    };
    for backend in BACKENDS {
        a.sweep(backend, &lengths, |k, rng| chain(backend, 3, k, rng), run);
    }
    for backend in [RATIONAL, FLOAT] {
        a.grid(
            backend,
            ("chain length", &lengths),
            ("dimension N", &[1.0, 2.0, 3.0, 4.0, 8.0]),
            |k, n, rng| chain(backend, n as usize, k, rng),
            run,
        );
    }

    by_dimension(
        a,
        s,
        "contraction",
        "krawczyk step",
        "One Krawczyk test for A x = b: K = c - Y(Ac - b) + (I - YA)(X - c) and K ⊂ int X, with the ∞-norm contraction margin of I - YA.",
        &Krawczyk,
    );
    by_shape(
        a,
        s,
        "contraction",
        "krawczyk step",
        "One Krawczyk test (shape applies to the box data; A stays narrow, Y stays a point matrix).",
        &Krawczyk,
    );
    by_magnitude(
        a,
        s,
        "contraction",
        "krawczyk step",
        "One Krawczyk test.",
        &Krawczyk,
    );

    dyadic_operation(
        a,
        s,
        "bisect axis",
        "Split a dyadic box at the midpoint of its widest axis (dimension 4 in the series; random shapes exclude points, which cannot be split).",
        &Bisect,
    );
    dyadic_operation(
        a,
        s,
        "regrid",
        "Round every lane outward to the grid 2^(e-16) (dimension 4 in the series).",
        &Regrid,
    );
    atlas.finish();
}
