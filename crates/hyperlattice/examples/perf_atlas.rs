//! Performance-atlas workloads over exact lattice carriers and generated entry recipes.
//!
//! Run through scripts/perf-atlas/run.sh, or from the workspace root with
//! `cargo run --release -p hyperlattice --example perf_atlas -- --out atlas.json`.
//! This example registers 44 operations; the shared harness selects names and may
//! stop/skip points using observed timing. Its fuzz series are deterministic seeded
//! samples, not a correctness-directed fuzzing campaign.
//!
//! Factories build rational recipe data outside timed callbacks. Callbacks rebuild
//! Real operands, including square roots, and ordinarily force outputs through a
//! sign precheck and scientific formatting. This does not reset global caches or
//! prove every call is cold. Certified zeros return a length token without formatting;
//! structural-fact and orientation rows return a Boolean/sign token instead. Tokens
//! are not exact-output checksums, topology evidence, or independent mathematical
//! oracles. The harness checks completion only on its first probe, not every repeat.
//!
//! Native Problem errors and undecided output signs collapse to None. A successful
//! Option only means this callback reached its terminal, not that an independent
//! expectation or certainty receipt was checked. The sign precision is not a total
//! work/time/memory limit; construction and subsequent formatting have no local
//! resource guard. The shared harness catches ordinary unwinding but cannot preempt
//! a stuck call. External containment is needed for expensive experiments.
//!
//! Names describe generator recipes, not guaranteed resulting representations:
//! rational reductions can shrink sizes, and the irrational recipe can be a square.
//! Some retained human-readable operation labels overstate ranges/error detail;
//! the generator and workload docs below give the actual contracts.
use hyperlattice::{
    Complex, Matrix3, Matrix4, Point3, Problem, ProjectivePlane3, Rational, Real, RealSign,
    Vector2, Vector3, Vector4, intersect_three_planes, intersect_two_planes, orient2_expr,
};
use perf_atlas::{Atlas, Rng, powers_of_two};

/// Four generator recipe labels; these neither enumerate Real storage classes nor guarantee the resulting structural kind.
const KINDS: [&str; 4] = ["integer", "rational", "dyadic", "irrational"];
/// Minimum binary refinement precision passed to sign queries; not an error tolerance, total work bound, or formatting limit.
const SIGN_PRECISION: i32 = -2048;
/// Sentinel 512 omitting perturbations in authored sweeps; it denotes zero noise, not the nonzero number 2^-512.
const SINGULAR: f64 = 512.0;

/// Recursive exact-entry recipe rebuilt into Real for each callback.
///
/// Keeps rational seeds outside timed work, with ordered sums/scales and optional
/// square roots inside. Recursive evaluation does not memoize shared subexpressions
/// or bound depth/work. Variants express recipes, not guaranteed Real classes.
#[derive(Clone, Debug)]
enum E {
    /// An exact rational seed, cloned when evaluated.
    Exact(#[doc = "Exact rational payload."] Rational),
    /// Principal sqrt(q), optionally negated; q is positive for generated fixtures but is not required to be a nonsquare.
    Root(
        #[doc = "Principal-root radicand; generated values are positive."] Rational,
        #[doc = "Whether to negate the principal root."] bool,
    ),
    /// Ordered sum of child recipes; an empty list evaluates to exact zero.
    Sum(#[doc = "Terms evaluated and reduced in stored order."] Vec<E>),
    /// A child recipe multiplied by an exact rational factor after recursive evaluation.
    Scaled(
        #[doc = "Child recipe evaluated before scaling."] Box<E>,
        #[doc = "Exact multiplicative factor."] Rational,
    ),
}

/// Parse an exact rational fixture string, panicking if the generated text is invalid; performs no floating-point ingress.
fn parse(text: &str) -> Rational {
    text.parse().expect("generated rational")
}

/// Set the last decimal digit's low bit, making a nonempty unsigned decimal integer odd.
///
/// For positive decimal ASCII integers this preserves bit length: an even integer increases by
/// one and an odd integer stays unchanged. Assumes a nonempty valid digit string;
/// malformed input is not validated. Used on Rng-generated positive integers.
fn odd(mut digits: String) -> String {
    let last = digits.pop().expect("nonempty digits") as u8 - b'0';
    digits.push(char::from(b'0' + (last | 1)));
    digits
}

/// Construct exact 2^exponent through rational integer power, including reciprocal powers; expect panics on a returned arithmetic error.
fn pow2(exponent: i64) -> Rational {
    Rational::new(2)
        .powi(exponent.into())
        .expect("power of two")
}

/// Consume a random sign, then construct a signed recipe with the requested pre-reduction size; delegates kind validation to signed_entry.
fn entry(kind: &str, bits: u32, rng: &mut Rng) -> E {
    let negative = rng.coin();
    signed_entry(kind, bits, negative, rng)
}

/// Construct the nonnegative-sign recipe for kind without drawing a sign coin; current generators produce strictly positive values.
fn positive(kind: &str, bits: u32, rng: &mut Rng) -> E {
    signed_entry(kind, bits, false, rng)
}

/// Construct an integer, ratio, dyadic, or signed-square-root recipe from positive random integers.
///
/// Clamp bits to at least one. Integer magnitude has that width; ratio numerator and
/// denominator start with bits and max(bits,2), then reduce. Dyadics use an odd
/// bits-wide numerator over 2^bits. Root radicands use an odd numerator over a
/// positive denominator, without excluding rational squares. Thus irrational is a
/// recipe label, not a proof. Unknown kinds panic via unreachable; no size budget
/// or unbiased-distribution guarantee is added to Rng.
fn signed_entry(kind: &str, bits: u32, negative: bool, rng: &mut Rng) -> E {
    let bits = bits.max(1);
    let sign = if negative { "-" } else { "" };
    match kind {
        "integer" => E::Exact(parse(&format!("{sign}{}", rng.decimal(bits)))),
        "rational" => E::Exact(parse(&format!(
            "{sign}{}/{}",
            rng.decimal(bits),
            rng.decimal(bits.max(2))
        ))),
        "dyadic" => {
            let numerator = parse(&format!("{sign}{}", odd(rng.decimal(bits))));
            E::Exact(&numerator / &pow2(i64::from(bits)))
        }
        "irrational" => E::Root(
            parse(&format!(
                "{}/{}",
                odd(rng.decimal(bits)),
                rng.decimal(bits.max(2))
            )),
            negative,
        ),
        other => unreachable!("unknown kind {other}"),
    }
}

/// Draw one of four kind labels and a width in 1..=max(max_bits,1), then a signed recipe; uses the shared remainder-based RNG.
fn mixed(max_bits: u32, rng: &mut Rng) -> E {
    let kind = KINDS[rng.below(4) as usize];
    let bits = rng.range(1, i64::from(max_bits.max(1))) as u32;
    entry(kind, bits, rng)
}

/// Build count independent recipes of a fixed kind/width, or draw kind/width per entry when None; allocates without a count/size bound.
fn entries(kind: Option<&str>, count: usize, bits: u32, rng: &mut Rng) -> Vec<E> {
    (0..count)
        .map(|_| match kind {
            Some(kind) => entry(kind, bits, rng),
            None => mixed(bits, rng),
        })
        .collect()
}

/// Recursively evaluate one recipe into an exact Real expression.
///
/// Clone exact rational data; take the principal root and then optionally negate;
/// reduce Sum left-to-right (empty means zero); evaluate Scaled before multiplying
/// by its rational factor. A root-domain error panics. Separate visits rebuild Real
/// expressions rather than memoizing recipe identity; later display is approximate.
fn real(e: &E) -> Real {
    match e {
        E::Exact(q) => Real::new(q.clone()),
        E::Root(q, negative) => {
            let root = Real::new(q.clone()).sqrt().expect("positive radicand");
            if *negative { -root } else { root }
        }
        E::Sum(terms) => terms
            .iter()
            .map(real)
            .reduce(|a, b| a + b)
            .unwrap_or_else(Real::zero),
        E::Scaled(inner, factor) => real(inner) * Real::new(factor.clone()),
    }
}

/// Return an exact rational-zero recipe, used for structural masks and omitted perturbations.
fn zero() -> E {
    E::Exact(Rational::zero())
}

/// Return an exact rational-one recipe, used for affine homogeneous entries and shifted arguments.
fn one() -> E {
    E::Exact(Rational::one())
}

/// Return a display-length token after an exact sign query at SIGN_PRECISION.
///
/// Undecided sign returns None; certified zero returns Some(1) without formatting.
/// Otherwise format .16e (one leading digit plus 16 fractional digits) and return
/// its byte length. The formatted approximation is discarded; length is not an
/// exact-value checksum. The sign cap does not bound subsequent formatting or the
/// total work of constructing/refining an expression.
fn digits(value: &Real) -> Option<usize> {
    match value.refine_sign_until(SIGN_PRECISION)? {
        RealSign::Zero => Some(1),
        _ => Some(format!("{value:.16e}").len()),
    }
}

/// Sum display-length tokens in iterator order, returning None at the first undecided sign.
///
/// Empty input returns Some(0). Later entries are not forced after failure. The usize
/// sum has no overflow guard or retained exact-output certificate.
fn force<'a>(values: impl IntoIterator<Item = &'a Real>) -> Option<usize> {
    values.into_iter().map(digits).sum()
}

/// Build a 3D point from the first three recipes; panics on a short slice and ignores trailing entries.
fn point3(e: &[E]) -> Point3 {
    Point3::new(real(&e[0]), real(&e[1]), real(&e[2]))
}

/// Force x, y, z in that order and sum their display-length tokens; stops at the first undecided sign.
fn force_point(p: &Point3) -> Option<usize> {
    force([&p.x, &p.y, &p.z])
}

/// Discard every native Problem into None; on Ok only, invoke the terminal callback and preserve its Option. No typed failure reason survives.
fn ok<T>(value: Result<T, Problem>, f: impl FnOnce(T) -> Option<usize>) -> Option<usize> {
    value.ok().and_then(f)
}

/// Private adapter routing the same workload closures to Vector2/3/4 native operations; it adds no geometry, policy, or independent validation.
trait Vector: Sized {
    /// Evaluate the first carrier-dimension recipes into lane order; short slices panic and extra recipes are ignored.
    fn build(e: &[E]) -> Self;
    /// Borrow the stored lanes in coordinate order without forcing their values.
    fn parts(&self) -> &[Real];
    /// Delegate borrowed componentwise addition to the native carrier.
    fn plus(&self, rhs: &Self) -> Self;
    /// Delegate owned vector-times-scalar multiplication to the native carrier.
    fn scaled(self, s: Real) -> Self;
    /// Delegate the exact dot expression to the native dimension-specific method.
    fn dot_with(&self, rhs: &Self) -> Real;
    /// Delegate the exact squared Euclidean norm expression.
    fn norm2(&self) -> Real;
    /// Delegate magnitude, retaining native square-root/domain errors until the caller erases them.
    fn length(&self) -> Result<Real, Problem>;
    /// Delegate ordinary normalization without adding an independent unit-length check.
    fn unit(&self) -> Result<Self, Problem>;
    /// Delegate structural nonzero-checked normalization, retaining its native rejection until the caller erases it.
    fn unit_checked(&self) -> Result<Self, Problem>;
    /// Delegate a+t*(rhs-a) interpolation/extrapolation; does not restrict t to the unit interval.
    fn interpolate(&self, rhs: &Self, t: &Real) -> Self;
    /// Force lanes in stored order, summing display lengths or stopping at the first undecided sign.
    fn force(&self) -> Option<usize> {
        force(self.parts())
    }
}

/// Implement the private Vector adapter by direct delegation for each listed tuple carrier; caller-provided slices must have enough lanes.
macro_rules! vector {
    ($($v:ident),*) => {$(
        impl Vector for $v {
            /// Evaluate the first carrier-dimension recipes into lane order; short slices panic and extra recipes are ignored.
            fn build(e: &[E]) -> Self {
                $v(std::array::from_fn(|i| real(&e[i])))
            }
            /// Borrow the stored lanes in coordinate order without forcing their values.
            fn parts(&self) -> &[Real] {
                &self.0
            }
            /// Delegate borrowed componentwise addition to the native carrier.
            fn plus(&self, rhs: &Self) -> Self {
                self + rhs
            }
            /// Delegate owned vector-times-scalar multiplication to the native carrier.
            fn scaled(self, s: Real) -> Self {
                self * s
            }
            /// Delegate the exact dot expression to the native dimension-specific method.
            fn dot_with(&self, rhs: &Self) -> Real {
                self.dot(rhs)
            }
            /// Delegate the exact squared Euclidean norm expression.
            fn norm2(&self) -> Real {
                self.norm_squared()
            }
            /// Delegate magnitude, retaining native square-root/domain errors until the caller erases them.
            fn length(&self) -> Result<Real, Problem> {
                self.magnitude()
            }
            /// Delegate ordinary normalization without adding an independent unit-length check.
            fn unit(&self) -> Result<Self, Problem> {
                self.normalize()
            }
            /// Delegate structural nonzero-checked normalization, retaining its native rejection until the caller erases it.
            fn unit_checked(&self) -> Result<Self, Problem> {
                self.normalize_checked()
            }
            /// Delegate a+t*(rhs-a) interpolation/extrapolation; does not restrict t to the unit interval.
            fn interpolate(&self, rhs: &Self, t: &Real) -> Self {
                self.lerp(rhs, t)
            }
        }
    )*};
}
vector!(Vector2, Vector3, Vector4);

/// Private adapter routing workload closures to Matrix3/4 and their matching vector carrier; does not add inversion/solve validation or certainty.
trait Matrix: Sized + Clone {
    /// Native vector carrier matching this matrix dimension.
    type V: Vector;
    /// Evaluate the first n*n recipes in row-major order; short slices panic and extra recipes are ignored.
    fn build(e: &[E]) -> Self;
    /// Borrow all stored entries as a row-major iterator without evaluating them.
    fn parts(&self) -> impl Iterator<Item = &Real>;
    /// Delegate borrowed entrywise addition to the native matrix carrier.
    fn plus(&self, rhs: &Self) -> Self;
    /// Delegate owned matrix-times-scalar multiplication.
    fn scaled(self, s: Real) -> Self;
    /// Delegate borrowed exact matrix multiplication.
    fn times(&self, rhs: &Self) -> Self;
    /// Delegate multiplication by the matching native vector carrier.
    fn apply(&self, v: &Self::V) -> Self::V;
    /// Delegate the exact determinant expression without an independent determinant oracle.
    fn det(&self) -> Real;
    /// Delegate ordinary consuming inversion; preserve its native Result until the caller erases it.
    fn inv(self) -> Result<Self, Problem>;
    /// Delegate consuming inversion with native structural nonzero checks; adds no independent inverse proof.
    fn inv_checked(self) -> Result<Self, Problem>;
    /// Delegate borrowed A/B to the native right-division solver; preserve its Result until the caller erases it.
    fn right_divide(&self, rhs: &Self) -> Result<Self, Problem>;
    /// Delegate native consuming signed integer power; this adapter adds no exponent validation or output oracle.
    fn power(self, exponent: i32) -> Result<Self, Problem>;
    /// Force entries in row-major order, summing display lengths or stopping at the first undecided sign.
    fn force(&self) -> Option<usize> {
        force(self.parts())
    }
}

/// Implement the private Matrix adapter by direct delegation for listed matrix/vector/dimension triples; row-major indexing is unchecked beyond normal slice bounds.
macro_rules! matrix {
    ($($m:ident $v:ident $n:literal),*) => {$(
        impl Matrix for $m {
            /// Native vector carrier matching this matrix dimension.
            type V = $v;
            /// Evaluate the first n*n recipes in row-major order; short slices panic and extra recipes are ignored.
            fn build(e: &[E]) -> Self {
                $m(std::array::from_fn(|r| std::array::from_fn(|c| real(&e[r * $n + c]))))
            }
            /// Borrow all stored entries as a row-major iterator without evaluating them.
            fn parts(&self) -> impl Iterator<Item = &Real> {
                self.0.iter().flatten()
            }
            /// Delegate borrowed entrywise addition to the native matrix carrier.
            fn plus(&self, rhs: &Self) -> Self {
                self + rhs
            }
            /// Delegate owned matrix-times-scalar multiplication.
            fn scaled(self, s: Real) -> Self {
                self * s
            }
            /// Delegate borrowed exact matrix multiplication.
            fn times(&self, rhs: &Self) -> Self {
                self * rhs
            }
            /// Delegate multiplication by the matching native vector carrier.
            fn apply(&self, v: &$v) -> $v {
                self * v
            }
            /// Delegate the exact determinant expression without an independent determinant oracle.
            fn det(&self) -> Real {
                self.determinant()
            }
            /// Delegate ordinary consuming inversion; preserve its native Result until the caller erases it.
            fn inv(self) -> Result<Self, Problem> {
                self.inverse()
            }
            /// Delegate consuming inversion with native structural nonzero checks; adds no independent inverse proof.
            fn inv_checked(self) -> Result<Self, Problem> {
                self.inverse_checked()
            }
            /// Delegate borrowed A/B to the native right-division solver; preserve its Result until the caller erases it.
            fn right_divide(&self, rhs: &Self) -> Result<Self, Problem> {
                self / rhs
            }
            /// Delegate native consuming signed integer power; this adapter adds no exponent validation or output oracle.
            fn power(self, exponent: i32) -> Result<Self, Problem> {
                self.powi(exponent)
            }
        }
    )*};
}
matrix!(Matrix3 Vector3 3, Matrix4 Vector4 4);

/// Dispatch 2 to Vector2, 3 to Vector3, and every other dimension to Vector4; this is a private fallback, not validation of arbitrary dimensions.
macro_rules! by_vector {
    ($n:expr, $v:ident => $body:expr) => {
        match $n {
            2 => {
                /// Local carrier alias selected by this macro branch; unsupported dimensions take the fallback branch.
                type $v = Vector2;
                $body
            }
            3 => {
                /// Local carrier alias selected by this macro branch; unsupported dimensions take the fallback branch.
                type $v = Vector3;
                $body
            }
            _ => {
                /// Local carrier alias selected by this macro branch; unsupported dimensions take the fallback branch.
                type $v = Vector4;
                $body
            }
        }
    };
}

/// Dispatch 3 to Matrix3 and every other dimension to Matrix4; callers supply supported dimensions, but the macro does not enforce that precondition.
macro_rules! by_matrix {
    ($n:expr, $m:ident => $body:expr) => {
        if $n == 3 {
            /// Local carrier alias selected by this macro branch; unsupported dimensions take the fallback branch.
            type $m = Matrix3;
            $body
        } else {
            /// Local carrier alias selected by this macro branch; unsupported dimensions take the fallback branch.
            type $m = Matrix4;
            $body
        }
    };
}

/// Owned exact recipes for two vector/complex operands and a scalar parameter; factories prepare these outside timed evaluation.
struct VecIn {
    /// First operand recipes in coordinate order.
    a: Vec<E>,
    /// Second operand recipes, even for callbacks that do not consume them.
    b: Vec<E>,
    /// Scalar recipe used for scaling or lerp; not clamped to `[0,1]`.
    t: E,
}

/// Generate two dim-lane recipe vectors and one scalar; individual callbacks may leave the second vector or scalar unused.
fn vec_in(kind: Option<&str>, dim: usize, bits: u32, rng: &mut Rng) -> VecIn {
    VecIn {
        a: entries(kind, dim, bits, rng),
        b: entries(kind, dim, bits, rng),
        t: entries(kind, 1, bits, rng).remove(0),
    }
}

/// Caller-supplied f64 axes interpreted as bit counts by workload factories; values are cast rather than validated as positive integers.
struct Settings {
    /// Main bit-count axis, normally powers of two from 1 through 4096.
    bits: Vec<f64>,
    /// Grid/mixed bit-count axis, normally powers of two from 1 through 1024.
    few_bits: Vec<f64>,
}

/// Register vector sweeps, optional dimension grids, and mixed seeded samples.
///
/// Requires nonempty dims. Prefer dimension 3 for four-kind bit sweeps, otherwise
/// use `dims[0]`. Multiple dimensions add rational/root bit-by-dimension grids.
/// Mixed sampling chooses one supplied dimension per fixture and independently
/// mixes lane kinds/sizes. Axis casts and callback semantics are not validated;
/// the shared harness controls filtering, timing stops, and first-probe status.
fn vector_operation(
    atlas: &mut Atlas,
    settings: &Settings,
    (name, description): (&str, &str),
    dims: &[usize],
    run: impl Fn(&VecIn) -> Option<usize>,
) {
    atlas.operation("vector", name, "entry bits", description);
    let main = if dims.contains(&3) { 3 } else { dims[0] };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &settings.bits,
            |m, rng| vec_in(Some(kind), main, m as u32, rng),
            &run,
        );
    }
    if dims.len() > 1 {
        let d: Vec<f64> = dims.iter().map(|&d| d as f64).collect();
        for kind in ["rational", "irrational"] {
            atlas.grid(
                kind,
                ("entry bits", &settings.few_bits),
                ("dimension", &d),
                |x, y, rng| vec_in(Some(kind), y as usize, x as u32, rng),
                &run,
            );
        }
    }
    atlas.fuzz(
        "mixed",
        &settings.few_bits,
        |m, rng| {
            let dim = dims[rng.below(dims.len() as u64) as usize];
            vec_in(None, dim, m as u32, rng)
        },
        &run,
    );
}

/// Owned row-major matrix recipes plus vector/scalar operands; some workload variants deliberately leave unused fields empty or zero.
struct MatIn {
    /// Selected matrix dimension, supplied as 3 or 4 by current factories.
    n: usize,
    /// First matrix recipes in row-major order, sometimes structured or row-dependent.
    a: Vec<E>,
    /// Second matrix recipes in row-major order, unused by unary callbacks.
    b: Vec<E>,
    /// Vector recipes for matrix-vector multiplication; may be empty in other workloads.
    v: Vec<E>,
    /// Scalar recipe for scaling; may be an unused zero recipe.
    s: E,
}

/// Generate two n*n matrices, an n-lane vector, and one scalar recipe; n/product size and allocation are not separately bounded.
fn mat_in(kind: Option<&str>, n: usize, bits: u32, rng: &mut Rng) -> MatIn {
    MatIn {
        n,
        a: entries(kind, n * n, bits, rng),
        b: entries(kind, n * n, bits, rng),
        v: entries(kind, n, bits, rng),
        s: entries(kind, 1, bits, rng).remove(0),
    }
}

/// Register four-kind 3x3 bit sweeps, rational/root 3x3/4x4 grids, and mixed samples.
///
/// Factories build both matrices, a vector, and a scalar even when the callback only
/// uses some fields. Random dense matrices are not certified nonsingular. Callback
/// Options erase native errors; shared-harness timing and selection limits apply.
fn matrix_operation(
    atlas: &mut Atlas,
    settings: &Settings,
    (name, description): (&str, &str),
    run: impl Fn(&MatIn) -> Option<usize>,
) {
    atlas.operation("matrix", name, "entry bits", description);
    for kind in KINDS {
        atlas.sweep(
            kind,
            &settings.bits,
            |m, rng| mat_in(Some(kind), 3, m as u32, rng),
            &run,
        );
    }
    for kind in ["rational", "irrational"] {
        atlas.grid(
            kind,
            ("entry bits", &settings.few_bits),
            ("dimension", &[3.0, 4.0]),
            |x, y, rng| mat_in(Some(kind), y as usize, x as u32, rng),
            &run,
        );
    }
    atlas.fuzz(
        "mixed",
        &settings.few_bits,
        |m, rng| {
            let n = 3 + rng.below(2) as usize;
            mat_in(None, n, m as u32, rng)
        },
        &run,
    );
}

/// Generate a rational matrix, then overwrite entries according to a structural mask.
///
/// Recognizes dense, upper triangular, diagonal, and affine; affine sets the last row
/// to (0,...,0,1). Unknown strings zero the last row but do not restore its final one.
/// Used with n=4; no shape validation or nonsingularity assertion is performed.
/// All original random entries are generated before applying the mask.
fn structured(structure: &str, n: usize, bits: u32, rng: &mut Rng) -> Vec<E> {
    let mut e = entries(Some("rational"), n * n, bits, rng);
    for r in 0..n {
        for c in 0..n {
            let keep = match structure {
                "dense" => true,
                "upper triangular" => c >= r,
                "diagonal" => c == r,
                // Affine: last row is (0, ..., 0, 1).
                _ => r + 1 < n,
            };
            if !keep {
                e[r * n + c] = zero();
            }
        }
    }
    if structure == "affine" {
        e[n * n - 1] = one();
    }
    e
}

/// Make the last row the sum of n-1 independently generated 64-bit recipe rows, plus optional noise.
///
/// For k<SINGULAR, add positive integer noise in 1..=7 times exact 2^-(k as i64)
/// per column. At the authored sentinel, omit noise, giving an exactly dependent
/// last row. A finite perturbation is not a nonsingularity or condition-number proof.
/// Used for n=3/4 and finite nonnegative integral k; no validation of other dimensions,
/// float casts, rank, or resource growth is supplied.
fn near_singular(kind: &str, n: usize, k: f64, rng: &mut Rng) -> Vec<E> {
    let mut e = entries(Some(kind), n * (n - 1), 64, rng);
    for c in 0..n {
        let mut terms: Vec<E> = (0..n - 1).map(|r| e[r * n + c].clone()).collect();
        if k < SINGULAR {
            let noise = Rational::new(rng.range(1, 7));
            terms.push(E::Scaled(Box::new(E::Exact(noise)), pow2(-(k as i64))));
        }
        e.push(E::Sum(terms));
    }
    e
}

/// Register 13 vector/point workload families over the configured bit axes.
///
/// Covers addition, scaling, dot/norm/length, ordinary/checked normalization, lerp,
/// 3D cross/angle/frame, 2D wedge, and 3D centroid. Lerp parameters are not restricted
/// to `[0,1]`. Angle/frame use checked native paths and can refuse inputs; no residual,
/// unit-length, or orthogonality oracle is added. Only the two returned frame vectors
/// are forced. Centroid sweeps use counts 1..4096 with 32-bit recipes; mixed samples
/// use counts 1..256 and widths up to 64. Point construction and centroid-owned work
/// occur inside the callback; unused recipe fields remain outside timed Real work.
fn vectors(atlas: &mut Atlas, s: &Settings) {
    vector_operation(
        atlas,
        s,
        (
            "vector add",
            "Componentwise sum a + b; dimension 3 in the series, 2-4 in grids.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => V::build(&x.a).plus(&V::build(&x.b)).force()),
    );
    vector_operation(
        atlas,
        s,
        (
            "vector scale",
            "Vector times a scalar of the same kind and size.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => V::build(&x.a).scaled(real(&x.t)).force()),
    );
    vector_operation(
        atlas,
        s,
        (
            "dot",
            "Exact dot product, approximated to 17 significant digits.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => digits(&V::build(&x.a).dot_with(&V::build(&x.b)))),
    );
    vector_operation(
        atlas,
        s,
        ("norm squared", "Exact squared Euclidean norm."),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => digits(&V::build(&x.a).norm2())),
    );
    vector_operation(
        atlas,
        s,
        (
            "magnitude",
            "Euclidean norm: square root of the exact squared norm.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => ok(V::build(&x.a).length(), |m| digits(&m))),
    );
    vector_operation(
        atlas,
        s,
        ("normalize", "Unit vector; every component approximated."),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => ok(V::build(&x.a).unit(), |u| u.force())),
    );
    vector_operation(
        atlas,
        s,
        (
            "normalize checked",
            "Unit vector after a structural nonzero proof of the norm; refusals are failures.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => ok(V::build(&x.a).unit_checked(), |u| u.force())),
    );
    vector_operation(
        atlas,
        s,
        (
            "lerp",
            "a + t (b - a) with a scalar t of the same kind and size.",
        ),
        &[2, 3, 4],
        |x| by_vector!(x.a.len(), V => V::build(&x.a).interpolate(&V::build(&x.b), &real(&x.t)).force()),
    );
    vector_operation(atlas, s, ("cross", "3D cross product."), &[3], |x| {
        force(&Vector3::build(&x.a).cross(&Vector3::build(&x.b)).0)
    });
    vector_operation(
        atlas,
        s,
        ("wedge", "2D wedge (signed parallelogram area)."),
        &[2],
        |x| digits(&Vector2::build(&x.a).wedge(&Vector2::build(&x.b))),
    );
    vector_operation(
        atlas,
        s,
        (
            "angle to",
            "acos of the dot product of the two normalized 3D vectors.",
        ),
        &[3],
        |x| {
            ok(Vector3::build(&x.a).angle_to(&Vector3::build(&x.b)), |a| {
                digits(&a)
            })
        },
    );
    vector_operation(
        atlas,
        s,
        (
            "orthonormal basis",
            "Two unit vectors completing a checked orthonormal frame for a 3D vector.",
        ),
        &[3],
        |x| {
            ok(
                Vector3::build(&x.a).orthonormal_basis_checked(),
                |(u, v)| Some(u.force()? + v.force()?),
            )
        },
    );

    atlas.operation(
        "vector",
        "point centroid",
        "point count",
        "Centroid of n 3D points with 32-bit coordinates.",
    );
    let counts = powers_of_two(0, 12);
    let centroid = |p: &Vec<Vec<E>>| {
        let points: Vec<Point3> = p.iter().map(|e| point3(e)).collect();
        Point3::centroid(&points).and_then(|c| force_point(&c))
    };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &counts,
            |m, rng| {
                (0..m as usize)
                    .map(|_| entries(Some(kind), 3, 32, rng))
                    .collect()
            },
            centroid,
        );
    }
    atlas.fuzz(
        "mixed",
        &powers_of_two(0, 8),
        |m, rng| (0..m as usize).map(|_| entries(None, 3, 64, rng)).collect(),
        centroid,
    );
}

/// Register 17 matrix workload families: eight generic, four structure, four conditioning, and power.
///
/// Generic callbacks force entries or determinant output after native operations.
/// Structure sweeps use 4x4 rational masks; right division is dense B divided by
/// structured A. Structural facts only return has_known_zero_lane as a token, not
/// formatted entries or an expected-fact assertion. Conditioning sweeps/grids omit
/// noise at k=512; mixed samples choose one kind for the whole matrix, not per lane.
/// Failures lose their native reason through ok. Power series use positive exponents
/// 1..1024 (no zero/negative case), 3x3 8-bit recipes, an integer bit grid, and mixed
/// 3x3/4x4 recipes up to 16 bits. No determinant, inverse, solve, or power oracle is
/// checked; singular random fixtures can remain failures.
fn matrices(atlas: &mut Atlas, s: &Settings) {
    matrix_operation(
        atlas,
        s,
        ("matrix add", "Entrywise sum of two square matrices."),
        |x| by_matrix!(x.n, M => M::build(&x.a).plus(&M::build(&x.b)).force()),
    );
    matrix_operation(
        atlas,
        s,
        (
            "matrix scale",
            "Matrix times a scalar of the same kind and size.",
        ),
        |x| by_matrix!(x.n, M => M::build(&x.a).scaled(real(&x.s)).force()),
    );
    matrix_operation(
        atlas,
        s,
        ("matrix multiply", "Exact matrix product A·B."),
        |x| by_matrix!(x.n, M => M::build(&x.a).times(&M::build(&x.b)).force()),
    );
    matrix_operation(
        atlas,
        s,
        ("matrix × vector", "Exact matrix-vector product A·v."),
        |x| by_matrix!(x.n, M => M::build(&x.a).apply(&<M as Matrix>::V::build(&x.v)).force()),
    );
    matrix_operation(
        atlas,
        s,
        ("determinant", "Exact determinant of a dense matrix."),
        |x| by_matrix!(x.n, M => digits(&M::build(&x.a).det())),
    );
    matrix_operation(
        atlas,
        s,
        (
            "inverse",
            "Adjugate/determinant inverse; an unknown-zero determinant is refined, a zero one fails.",
        ),
        |x| by_matrix!(x.n, M => ok(M::build(&x.a).inv(), |m| m.force())),
    );
    matrix_operation(
        atlas,
        s,
        (
            "inverse checked",
            "Inverse that refuses a determinant not structurally known to be nonzero; refusals are failures.",
        ),
        |x| by_matrix!(x.n, M => ok(M::build(&x.a).inv_checked(), |m| m.force())),
    );
    matrix_operation(
        atlas,
        s,
        (
            "right divide (solve)",
            "A / B = A·B⁻¹ through the fixed-size left solver.",
        ),
        |x| by_matrix!(x.n, M => ok(M::build(&x.a).right_divide(&M::build(&x.b)), |m| m.force())),
    );

    for (name, run) in [
        (
            "determinant by structure",
            (|x: &MatIn| by_matrix!(x.n, M => digits(&M::build(&x.a).det())))
                as fn(&MatIn) -> Option<usize>,
        ),
        (
            "inverse by structure",
            |x: &MatIn| by_matrix!(x.n, M => ok(M::build(&x.a).inv(), |m| m.force())),
        ),
        (
            "right divide by structure",
            |x: &MatIn| by_matrix!(x.n, M => ok(M::build(&x.b).right_divide(&M::build(&x.a)), |m| m.force())),
        ),
        ("structural facts", |x: &MatIn| {
            // Includes rebuilding the entries; the fact scan itself is the rest.
            let facts = Matrix4::build(&x.a).structural_facts();
            Some(usize::from(facts.has_known_zero_lane()))
        }),
    ] {
        atlas.operation(
            "matrix",
            name,
            "entry bits",
            "4x4 rational matrices whose zero pattern is dense, affine (last row 0,0,0,1), upper triangular or diagonal; shows structural fast paths. Right division divides a dense matrix by the structured one.",
        );
        for structure in ["dense", "affine", "upper triangular", "diagonal"] {
            atlas.sweep(
                structure,
                &s.bits,
                |m, rng| MatIn {
                    n: 4,
                    a: structured(structure, 4, m as u32, rng),
                    b: entries(Some("rational"), 16, m as u32, rng),
                    v: Vec::new(),
                    s: zero(),
                },
                run,
            );
        }
    }

    let ks: Vec<f64> = (0..=15)
        .map(|i| f64::from(i * 32))
        .chain([SINGULAR])
        .collect();
    /// Function pointer for a conditioning callback returning only a terminal token or erased failure.
    type Conditioned = fn(&MatIn) -> Option<usize>;
    let conditioned: [(&str, &str, Conditioned); 4] = [
        (
            "determinant near singular",
            "Determinant of a 3x3 matrix whose last row is the sum of the others plus 2^-k·noise (64-bit entries); the last point (k = 512) is exactly singular.",
            |x| by_matrix!(x.n, M => digits(&M::build(&x.a).det())),
        ),
        (
            "inverse near singular",
            "Inverse of the same near-singular matrices; the exactly singular point fails.",
            |x| by_matrix!(x.n, M => ok(M::build(&x.a).inv(), |m| m.force())),
        ),
        (
            "inverse checked near singular",
            "Checked inverse of the same near-singular matrices; unknown-zero determinants are refused.",
            |x| by_matrix!(x.n, M => ok(M::build(&x.a).inv_checked(), |m| m.force())),
        ),
        (
            "solve near singular",
            "Right division B / A by the same near-singular matrices A.",
            |x| by_matrix!(x.n, M => ok(M::build(&x.b).right_divide(&M::build(&x.a)), |m| m.force())),
        ),
    ];
    for (name, description, run) in conditioned {
        atlas.operation(
            "conditioning",
            name,
            "perturbation exponent k (ε = 2^-k; 512 = singular)",
            description,
        );
        let make = |kind: &'static str, n: usize| {
            move |k: f64, rng: &mut Rng| MatIn {
                n,
                a: near_singular(kind, n, k, rng),
                b: entries(Some(kind), n * n, 64, rng),
                v: Vec::new(),
                s: zero(),
            }
        };
        for kind in ["integer", "rational", "dyadic", "irrational"] {
            atlas.sweep(kind, &ks, make(kind, 3), run);
        }
        for kind in ["rational", "irrational"] {
            atlas.grid(
                kind,
                ("perturbation exponent k", &ks),
                ("dimension", &[3.0, 4.0]),
                |k, n, rng| make(kind, n as usize)(k, rng),
                run,
            );
        }
        atlas.fuzz(
            "mixed",
            &ks,
            |k, rng| {
                let kind = KINDS[rng.below(4) as usize];
                make(kind, 3 + rng.below(2) as usize)(k, rng)
            },
            run,
        );
    }

    atlas.operation(
        "matrix",
        "matrix power",
        "exponent",
        "Integer power of a 3x3 matrix with 8-bit entries, by repeated squaring.",
    );
    let exponents = powers_of_two(0, 10);
    let power =
        |(x, e): &(MatIn, i32)| by_matrix!(x.n, M => ok(M::build(&x.a).power(*e), |m| m.force()));
    for kind in KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |e, rng| (mat_in(Some(kind), 3, 8, rng), e as i32),
            power,
        );
    }
    atlas.grid(
        "integer",
        ("exponent", &exponents),
        ("entry bits", &powers_of_two(0, 6)),
        |e, b, rng| (mat_in(Some("integer"), 3, b as u32, rng), e as i32),
        power,
    );
    atlas.fuzz(
        "mixed",
        &exponents,
        |e, rng| (mat_in(None, 3 + rng.below(2) as usize, 16, rng), e as i32),
        power,
    );
}

/// Register batch point transformation, axis-angle rotation, and rotation-between-vector workloads.
///
/// Batch sweeps force all returned coordinates of 1..4096 points under an affine
/// matrix with 32-bit recipes; mixed samples use unrestricted projective matrices,
/// 1..256 points, and widths up to 64. Failure of native transformation or forcing
/// returns None, without an independent transform equation check.
///
/// The angle recipe is an odd 53-bit integer divided by 2^54, hence [1/4,1/2)
/// radians, despite the retained description saying [1/2,1). Axis-angle timing
/// constructs the axis, angle, point, matrix, and transformed point, forcing only
/// the point output. Between-vector timing forces the returned matrix; no rotation
/// orthogonality or mapping oracle is asserted.
fn transforms(atlas: &mut Atlas, s: &Settings) {
    atlas.operation(
        "transform",
        "transform point batch",
        "batch size",
        "Matrix4::transform_point3_batch with an affine matrix and points of 32-bit entries; every output coordinate approximated.",
    );
    let sizes = powers_of_two(0, 12);
    let batch = |(m, points): &(Vec<E>, Vec<Vec<E>>)| {
        let matrix = Matrix4::build(m);
        let points: Vec<Point3> = points.iter().map(|p| point3(p)).collect();
        ok(matrix.transform_point3_batch(&points), |out| {
            out.iter().map(force_point).sum()
        })
    };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                let mut m = entries(Some(kind), 16, 32, rng);
                for (i, slot) in m.iter_mut().enumerate().skip(12) {
                    *slot = if i == 15 { one() } else { zero() };
                }
                (
                    m,
                    (0..n as usize)
                        .map(|_| entries(Some(kind), 3, 32, rng))
                        .collect(),
                )
            },
            batch,
        );
    }
    atlas.fuzz(
        "mixed (projective)",
        &powers_of_two(0, 8),
        |n, rng| {
            (
                entries(None, 16, 64, rng),
                (0..n as usize).map(|_| entries(None, 3, 64, rng)).collect(),
            )
        },
        batch,
    );

    atlas.operation(
        "transform",
        "rotate point (axis-angle)",
        "entry bits",
        "Build Matrix4::rotation_axis_angle about a random axis by a 53-bit dyadic angle in [1/2, 1), then transform a point; sin/cos and normalization are lazy until the output is approximated.",
    );
    let rotate = |(axis, angle, p): &(Vec<E>, E, Vec<E>)| {
        ok(
            Matrix4::rotation_axis_angle(&Vector3::build(axis), real(angle)),
            |m| ok(m.transform_point3(&point3(p)), |q| force_point(&q)),
        )
    };
    let angle = |rng: &mut Rng| E::Exact(&parse(&odd(rng.decimal(53))) / &pow2(54));
    for kind in KINDS {
        atlas.sweep(
            kind,
            &s.bits,
            |m, rng| {
                (
                    entries(Some(kind), 3, m as u32, rng),
                    angle(rng),
                    entries(Some(kind), 3, m as u32, rng),
                )
            },
            rotate,
        );
    }
    atlas.fuzz(
        "mixed",
        &s.few_bits,
        |m, rng| {
            (
                entries(None, 3, m as u32, rng),
                angle(rng),
                entries(None, 3, m as u32, rng),
            )
        },
        rotate,
    );

    atlas.operation(
        "transform",
        "rotation between vectors",
        "entry bits",
        "Matrix4::rotation_between_vectors for two random 3D vectors, all entries approximated.",
    );
    let between = |x: &VecIn| {
        ok(
            Matrix4::rotation_between_vectors(&Vector3::build(&x.a), &Vector3::build(&x.b)),
            |m| m.force(),
        )
    };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &s.bits,
            |m, rng| vec_in(Some(kind), 3, m as u32, rng),
            between,
        );
    }
    atlas.fuzz(
        "mixed",
        &s.few_bits,
        |m, rng| vec_in(None, 3, m as u32, rng),
        between,
    );
}

/// Register eight complex workload families with terminal component forcing.
///
/// Four binary callbacks include reciprocal, which still builds its unused second
/// operand. Power uses positive 1..1024 in sweeps and either sign in mixed samples.
/// Atanh and Jacobi sn/cn/dn (m=1/2) use separately generated positive fractions plus
/// one, scaled by 2^k. Only dyadic fractions guarantee shifted values in [1.5,2);
/// 53-bit rational fractions lie in (1/2,2), and root fractions in (sqrt(1/2),sqrt(2)).
/// Thus the retained general u,v in [1,2) description is too narrow. The argument
/// exponent is a scale recipe, not a certified |z| or conditioning measurement.
///
/// Near-cut atanh uses x=1+positive recipe and signed shrinking imaginary part,
/// omitted at k=512. Always x>1, but x need not be <=2. Native cut errors and
/// undecided signs become None; the report cannot distinguish NotANumber from
/// Exhausted as its description suggests. No branch identity or Jacobi residual
/// oracle is checked. Each mixed unary sample picks one kind for both components.
fn complex(atlas: &mut Atlas, s: &Settings) {
    let c = |e: &[E]| Complex::new(real(&e[0]), real(&e[1]));
    let force_c = |z: &Complex| force([&z.re, &z.im]);
    /// Complex binary operation pointer preserving Problem until the outer ok adapter erases it.
    type Binary = fn(Complex, Complex) -> Result<Complex, Problem>;
    let binary: [(&str, &str, Binary); 4] = [
        ("complex add", "Exact complex sum.", |a, b| Ok(a + b)),
        ("complex multiply", "Exact complex product.", |a, b| {
            Ok(a * b)
        }),
        ("complex divide", "Exact complex quotient.", |a, b| a / b),
        (
            "complex reciprocal",
            "1 / a (second operand unused).",
            |a, _| a.reciprocal(),
        ),
    ];
    for (name, description, op) in binary {
        atlas.operation("complex", name, "operand bits", description);
        let run = |x: &VecIn| ok(op(c(&x.a), c(&x.b)), |z| force_c(&z));
        for kind in KINDS {
            atlas.sweep(
                kind,
                &s.bits,
                |m, rng| vec_in(Some(kind), 2, m as u32, rng),
                run,
            );
        }
        atlas.grid(
            "rational",
            ("left operand bits", &s.few_bits),
            ("right operand bits", &s.few_bits),
            |x, y, rng| VecIn {
                a: entries(Some("rational"), 2, x as u32, rng),
                b: entries(Some("rational"), 2, y as u32, rng),
                t: zero(),
            },
            run,
        );
        atlas.fuzz(
            "mixed",
            &s.few_bits,
            |m, rng| vec_in(None, 2, m as u32, rng),
            run,
        );
    }

    atlas.operation(
        "complex",
        "complex power",
        "exponent",
        "Integer power of a complex value with 16-bit parts; fuzzed exponents take either sign.",
    );
    let exponents = powers_of_two(0, 10);
    let power = |(z, e): &(Vec<E>, i64)| ok(c(z).powi(*e), |w| force_c(&w));
    for kind in KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |e, rng| (entries(Some(kind), 2, 16, rng), e as i64),
            power,
        );
    }
    atlas.fuzz(
        "mixed",
        &exponents,
        |e, rng| {
            (
                entries(None, 2, 16, rng),
                -(e as i64) * (rng.below(2) as i64 * 2 - 1),
            )
        },
        power,
    );

    // z = 2^k * ((1 + positive recipe) + i*(1 + positive recipe)).
    // Rational/root recipes need not keep either shifted part below two.
    let scaled = |kind: &str, k: i32, rng: &mut Rng| -> Vec<E> {
        (0..2)
            .map(|_| {
                let fraction = positive(kind, 53, rng);
                E::Scaled(Box::new(E::Sum(vec![fraction, one()])), pow2(i64::from(k)))
            })
            .collect()
    };
    /// Complex unary callback pointer returning a forced-output length token or erased failure.
    type Unary = fn(Complex) -> Option<usize>;
    /// Force real then imaginary components of an owned complex result and return their summed display lengths.
    fn force_parts(z: Complex) -> Option<usize> {
        force([&z.re, &z.im])
    }
    let unary: [(&str, Unary, i32, i32); 2] = [
        ("complex atanh", |z| ok(z.atanh(), force_parts), -32, 32),
        (
            "complex jacobi sncndn (m = 1/2)",
            |z| {
                ok(
                    z.jacobi_sncndn(&Real::new(Rational::fraction(1, 2).unwrap())),
                    |(a, b, c)| Some(force_parts(a)? + force_parts(b)? + force_parts(c)?),
                )
            },
            -24,
            4,
        ),
    ];
    for (name, op, lo, hi) in unary {
        let range: Vec<f64> = (lo..=hi).step_by(2).map(f64::from).collect();
        atlas.operation(
            "complex",
            name,
            "argument exponent (|z| ≈ 2^k)",
            &format!("{name} of z = 2^k (u + iv), u, v in [1, 2); all output parts approximated."),
        );
        for kind in ["dyadic", "rational", "irrational"] {
            atlas.sweep(
                kind,
                &range,
                |k, rng| scaled(kind, k as i32, rng),
                |z| op(c(z)),
            );
        }
        atlas.fuzz(
            "mixed",
            &range,
            |k, rng| scaled(KINDS[1 + rng.below(3) as usize], k as i32, rng),
            |z| op(c(z)),
        );
    }

    atlas.operation(
        "complex",
        "complex atanh near branch cut",
        "imaginary-part exponent k (z = x + i·2^-k·y; 512 = on the cut)",
        "atanh of z = x + i·2^-k·y with x in (1, 2] on the real cut; the last point lies on the cut (NotANumber). Undecided cut membership is reported as Exhausted.",
    );
    let ks: Vec<f64> = (0..=15)
        .map(|i| f64::from(i * 32))
        .chain([SINGULAR])
        .collect();
    let near_cut = |kind: &str, k: f64, rng: &mut Rng| -> Vec<E> {
        let x = E::Sum(vec![positive(kind, 53, rng), one()]);
        let y = if k >= SINGULAR {
            zero()
        } else {
            E::Scaled(Box::new(entry(kind, 53, rng)), pow2(-(k as i64)))
        };
        vec![x, y]
    };
    for kind in ["dyadic", "rational", "irrational"] {
        atlas.sweep(
            kind,
            &ks,
            |k, rng| near_cut(kind, k, rng),
            |z| ok(c(z).atanh(), force_parts),
        );
    }
}

/// Register two homogeneous plane-intersection workloads and one orientation-sign family.
///
/// Three-plane timing constructs the homogeneous point, converts to affine, then
/// forces four homogeneous and three affine coordinates; failed conversion skips
/// forcing. Two-plane timing forces direction and moment but does not check a
/// nonzero line, Pluecker relation, or incidence residual. Rank-deficient inputs
/// are not excluded or independently classified.
///
/// Orientation constructs c=(2a+b)/3 + epsilon*left_perp(b-a) with epsilon=2^-k,
/// omitting perturbation at k=512. Mathematically its determinant is
///  epsilon*|b-a|^2; coincident a,b also give zero before the sentinel. The callback
/// returns a certified RealSign cast to u8, not a formatted scalar or proof-route
/// receipt, and does not assert the expected sign. Mixed samples choose one kind
/// for all coordinates and a width in 1..=256. These are expression/forcing timings,
/// not a region/topology or general predicate-policy qualification.
fn geometry(atlas: &mut Atlas, s: &Settings) {
    let plane = |e: &[E]| ProjectivePlane3::new(point3(e), real(&e[3]));
    atlas.operation(
        "projective",
        "intersect three planes",
        "coefficient bits",
        "Homogeneous intersection point of three planes, then its affine point; every coordinate approximated.",
    );
    let three = |e: &Vec<E>| {
        let p = intersect_three_planes(&plane(&e[0..4]), &plane(&e[4..8]), &plane(&e[8..12]));
        ok(p.to_affine_point(), |a| {
            Some(force([&p.x, &p.y, &p.z, &p.w])? + force_point(&a)?)
        })
    };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &s.bits,
            |m, rng| entries(Some(kind), 12, m as u32, rng),
            three,
        );
    }
    atlas.fuzz(
        "mixed",
        &s.few_bits,
        |m, rng| entries(None, 12, m as u32, rng),
        three,
    );

    atlas.operation(
        "projective",
        "intersect two planes",
        "coefficient bits",
        "Plücker line of two planes; direction and moment approximated.",
    );
    let two = |e: &Vec<E>| {
        let line = intersect_two_planes(&plane(&e[0..4]), &plane(&e[4..8]));
        Some(force_point(&line.direction)? + force_point(&line.moment)?)
    };
    for kind in KINDS {
        atlas.sweep(
            kind,
            &s.bits,
            |m, rng| entries(Some(kind), 8, m as u32, rng),
            two,
        );
    }
    atlas.fuzz(
        "mixed",
        &s.few_bits,
        |m, rng| entries(None, 8, m as u32, rng),
        two,
    );

    atlas.operation(
        "predicates",
        "orient2 sign near collinear",
        "perturbation exponent k (ε = 2^-k; 512 = collinear)",
        "Certified sign of orient2(a, b, c) with c = a + (b - a)/3 + ε·perp(b - a) and 64-bit coordinates. The sign is refined to 2^-2048; undecided signs are failures.",
    );
    let ks: Vec<f64> = (0..=15)
        .map(|i| f64::from(i * 32))
        .chain([SINGULAR])
        .collect();
    let collinear = |kind: &str, bits: u32, k: f64, rng: &mut Rng| -> Vec<E> {
        let p = entries(Some(kind), 4, bits, rng);
        let third = Rational::fraction(1, 3).unwrap();
        let two_thirds = Rational::fraction(2, 3).unwrap();
        let mut c = Vec::new();
        for axis in 0..2 {
            let (other, sign) = if axis == 0 { (1, -1) } else { (0, 1) };
            let mut terms = vec![
                E::Scaled(Box::new(p[axis].clone()), two_thirds.clone()),
                E::Scaled(Box::new(p[2 + axis].clone()), third.clone()),
            ];
            if k < SINGULAR {
                let eps = pow2(-(k as i64)) * Rational::new(sign);
                terms.push(E::Scaled(Box::new(p[2 + other].clone()), eps.clone()));
                terms.push(E::Scaled(Box::new(p[other].clone()), -eps));
            }
            c.push(E::Sum(terms));
        }
        p.into_iter().chain(c).collect()
    };
    let orient = |e: &Vec<E>| {
        let r: Vec<Real> = e.iter().map(real).collect();
        orient2_expr([&r[0], &r[1]], [&r[2], &r[3]], [&r[4], &r[5]])
            .refine_sign_until(SIGN_PRECISION)
            .map(|sign| sign as u8)
    };
    for kind in KINDS {
        atlas.sweep(kind, &ks, |k, rng| collinear(kind, 64, k, rng), orient);
    }
    for kind in ["rational", "irrational"] {
        atlas.grid(
            kind,
            ("perturbation exponent k", &ks),
            ("coordinate bits", &s.few_bits),
            |k, b, rng| collinear(kind, b as u32, k, rng),
            orient,
        );
    }
    atlas.fuzz(
        "mixed",
        &ks,
        |k, rng| {
            let kind = KINDS[rng.below(4) as usize];
            collinear(kind, rng.range(1, 256) as u32, k, rng)
        },
        orient,
    );
}

/// Parse shared atlas CLI options, register all workload families, and serialize the report.
///
/// Bit axes are powers of two from 1..4096 and 1..1024, with other count/exponent
/// axes declared by each family. Calls vectors, matrices, transforms, complex, then
/// geometry in that order. Filters and --quick affect the shared harness, not the
/// mathematical recipe definitions. finish writes stdout or creates/truncates the
/// requested output path and can panic on I/O failure; no completed-workload or
/// exact replay certificate is produced.
fn main() {
    let mut atlas = Atlas::from_args("hyperlattice");
    let settings = Settings {
        bits: powers_of_two(0, 12),
        few_bits: powers_of_two(0, 10),
    };
    vectors(&mut atlas, &settings);
    matrices(&mut atlas, &settings);
    transforms(&mut atlas, &settings);
    complex(&mut atlas, &settings);
    geometry(&mut atlas, &settings);
    atlas.finish();
}
