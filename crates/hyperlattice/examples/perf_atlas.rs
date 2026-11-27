//! Performance atlas for Hyperlattice: exact vectors, matrices, transforms,
//! complex scalars and projective constructions over Hyperreal entries, swept
//! across entry representations, entry sizes, dimensions, structure and
//! conditioning. Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperlattice --example perf_atlas -- --out atlas.json`.
//!
//! Every timed call rebuilds its operands from exact rationals (irrational
//! entries take a square root inside the call, so no approximation cached by an
//! earlier call is reused) and forces every output entry to 17 significant
//! digits. Before formatting, an entry's sign is certified down to
//! `2^-2048`; an entry whose sign stays undecided is reported as a failed call
//! rather than refined forever, because formatting an unproven zero does not
//! terminate.
use hyperlattice::{
    Complex, Matrix3, Matrix4, Point3, Problem, ProjectivePlane3, Rational, Real, RealSign,
    Vector2, Vector3, Vector4, intersect_three_planes, intersect_two_planes, orient2_expr,
};
use perf_atlas::{Atlas, Rng, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const KINDS: [&str; 4] = ["integer", "rational", "dyadic", "irrational"];
/// Signs are certified to this binary precision before an entry is formatted.
const SIGN_PRECISION: i32 = -2048;
/// Perturbation exponent that stands for an exactly singular input.
const SINGULAR: f64 = 512.0;

/// An exact recipe for one entry, turned into a [`Real`] inside timed calls.
#[derive(Clone, Debug)]
enum E {
    Exact(Rational),
    /// `±sqrt(q)` for a positive rational `q` that is not a square.
    Root(Rational, bool),
    Sum(Vec<E>),
    Scaled(Box<E>, Rational),
}

fn parse(text: &str) -> Rational {
    text.parse().expect("generated rational")
}

/// Make a decimal string odd without changing its bit length.
fn odd(mut digits: String) -> String {
    let last = digits.pop().expect("nonempty digits") as u8 - b'0';
    digits.push(char::from(b'0' + (last | 1)));
    digits
}

fn pow2(exponent: i64) -> Rational {
    Rational::new(2)
        .powi(exponent.into())
        .expect("power of two")
}

/// One entry of `kind` whose numerator and denominator have about `bits` bits.
fn entry(kind: &str, bits: u32, rng: &mut Rng) -> E {
    let negative = rng.coin();
    signed_entry(kind, bits, negative, rng)
}

fn positive(kind: &str, bits: u32, rng: &mut Rng) -> E {
    signed_entry(kind, bits, false, rng)
}

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

/// An entry of a random kind with a random size up to `max_bits`.
fn mixed(max_bits: u32, rng: &mut Rng) -> E {
    let kind = KINDS[rng.below(4) as usize];
    let bits = rng.range(1, i64::from(max_bits.max(1))) as u32;
    entry(kind, bits, rng)
}

/// `kind` is `None` for fuzzed inputs with a random kind and size per entry.
fn entries(kind: Option<&str>, count: usize, bits: u32, rng: &mut Rng) -> Vec<E> {
    (0..count)
        .map(|_| match kind {
            Some(kind) => entry(kind, bits, rng),
            None => mixed(bits, rng),
        })
        .collect()
}

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

fn zero() -> E {
    E::Exact(Rational::zero())
}

fn one() -> E {
    E::Exact(Rational::one())
}

/// Approximate to 17 significant digits after certifying the sign.
fn digits(value: &Real) -> Option<usize> {
    match value.refine_sign_until(SIGN_PRECISION)? {
        RealSign::Zero => Some(1),
        _ => Some(format!("{value:.16e}").len()),
    }
}

fn force<'a>(values: impl IntoIterator<Item = &'a Real>) -> Option<usize> {
    values.into_iter().map(digits).sum()
}

fn point3(e: &[E]) -> Point3 {
    Point3::new(real(&e[0]), real(&e[1]), real(&e[2]))
}

fn force_point(p: &Point3) -> Option<usize> {
    force([&p.x, &p.y, &p.z])
}

fn ok<T>(value: Result<T, Problem>, f: impl FnOnce(T) -> Option<usize>) -> Option<usize> {
    value.ok().and_then(f)
}

trait Vector: Sized {
    fn build(e: &[E]) -> Self;
    fn parts(&self) -> &[Real];
    fn plus(&self, rhs: &Self) -> Self;
    fn scaled(self, s: Real) -> Self;
    fn dot_with(&self, rhs: &Self) -> Real;
    fn norm2(&self) -> Real;
    fn length(&self) -> Result<Real, Problem>;
    fn unit(&self) -> Result<Self, Problem>;
    fn unit_checked(&self) -> Result<Self, Problem>;
    fn interpolate(&self, rhs: &Self, t: &Real) -> Self;
    fn force(&self) -> Option<usize> {
        force(self.parts())
    }
}

macro_rules! vector {
    ($($v:ident),*) => {$(
        impl Vector for $v {
            fn build(e: &[E]) -> Self {
                $v(std::array::from_fn(|i| real(&e[i])))
            }
            fn parts(&self) -> &[Real] {
                &self.0
            }
            fn plus(&self, rhs: &Self) -> Self {
                self + rhs
            }
            fn scaled(self, s: Real) -> Self {
                self * s
            }
            fn dot_with(&self, rhs: &Self) -> Real {
                self.dot(rhs)
            }
            fn norm2(&self) -> Real {
                self.norm_squared()
            }
            fn length(&self) -> Result<Real, Problem> {
                self.magnitude()
            }
            fn unit(&self) -> Result<Self, Problem> {
                self.normalize()
            }
            fn unit_checked(&self) -> Result<Self, Problem> {
                self.normalize_checked()
            }
            fn interpolate(&self, rhs: &Self, t: &Real) -> Self {
                self.lerp(rhs, t)
            }
        }
    )*};
}
vector!(Vector2, Vector3, Vector4);

trait Matrix: Sized + Clone {
    type V: Vector;
    fn build(e: &[E]) -> Self;
    fn parts(&self) -> impl Iterator<Item = &Real>;
    fn plus(&self, rhs: &Self) -> Self;
    fn scaled(self, s: Real) -> Self;
    fn times(&self, rhs: &Self) -> Self;
    fn apply(&self, v: &Self::V) -> Self::V;
    fn det(&self) -> Real;
    fn inv(self) -> Result<Self, Problem>;
    fn inv_checked(self) -> Result<Self, Problem>;
    fn right_divide(&self, rhs: &Self) -> Result<Self, Problem>;
    fn power(self, exponent: i32) -> Result<Self, Problem>;
    fn force(&self) -> Option<usize> {
        force(self.parts())
    }
}

macro_rules! matrix {
    ($($m:ident $v:ident $n:literal),*) => {$(
        impl Matrix for $m {
            type V = $v;
            fn build(e: &[E]) -> Self {
                $m(std::array::from_fn(|r| std::array::from_fn(|c| real(&e[r * $n + c]))))
            }
            fn parts(&self) -> impl Iterator<Item = &Real> {
                self.0.iter().flatten()
            }
            fn plus(&self, rhs: &Self) -> Self {
                self + rhs
            }
            fn scaled(self, s: Real) -> Self {
                self * s
            }
            fn times(&self, rhs: &Self) -> Self {
                self * rhs
            }
            fn apply(&self, v: &$v) -> $v {
                self * v
            }
            fn det(&self) -> Real {
                self.determinant()
            }
            fn inv(self) -> Result<Self, Problem> {
                self.inverse()
            }
            fn inv_checked(self) -> Result<Self, Problem> {
                self.inverse_checked()
            }
            fn right_divide(&self, rhs: &Self) -> Result<Self, Problem> {
                self / rhs
            }
            fn power(self, exponent: i32) -> Result<Self, Problem> {
                self.powi(exponent)
            }
        }
    )*};
}
matrix!(Matrix3 Vector3 3, Matrix4 Vector4 4);

macro_rules! by_vector {
    ($n:expr, $v:ident => $body:expr) => {
        match $n {
            2 => {
                type $v = Vector2;
                $body
            }
            3 => {
                type $v = Vector3;
                $body
            }
            _ => {
                type $v = Vector4;
                $body
            }
        }
    };
}

macro_rules! by_matrix {
    ($n:expr, $m:ident => $body:expr) => {
        if $n == 3 {
            type $m = Matrix3;
            $body
        } else {
            type $m = Matrix4;
            $body
        }
    };
}

struct VecIn {
    a: Vec<E>,
    b: Vec<E>,
    t: E,
}

fn vec_in(kind: Option<&str>, dim: usize, bits: u32, rng: &mut Rng) -> VecIn {
    VecIn {
        a: entries(kind, dim, bits, rng),
        b: entries(kind, dim, bits, rng),
        t: entries(kind, 1, bits, rng).remove(0),
    }
}

struct Settings {
    bits: Vec<f64>,
    few_bits: Vec<f64>,
}

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

struct MatIn {
    n: usize,
    a: Vec<E>,
    b: Vec<E>,
    v: Vec<E>,
    s: E,
}

fn mat_in(kind: Option<&str>, n: usize, bits: u32, rng: &mut Rng) -> MatIn {
    MatIn {
        n,
        a: entries(kind, n * n, bits, rng),
        b: entries(kind, n * n, bits, rng),
        v: entries(kind, n, bits, rng),
        s: entries(kind, 1, bits, rng).remove(0),
    }
}

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

/// A dense matrix shaped into `structure`, from rational entries.
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

/// Rows 0..n-1 are random; the last row is the sum of the others plus
/// `2^-k` times small integers, so `k = SINGULAR` is exactly singular.
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

fn complex(atlas: &mut Atlas, s: &Settings) {
    let c = |e: &[E]| Complex::new(real(&e[0]), real(&e[1]));
    let force_c = |z: &Complex| force([&z.re, &z.im]);
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

    // z = 2^k (u + i v) with 53-bit dyadic u, v in [1, 2).
    let scaled = |kind: &str, k: i32, rng: &mut Rng| -> Vec<E> {
        (0..2)
            .map(|_| {
                let fraction = positive(kind, 53, rng);
                E::Scaled(Box::new(E::Sum(vec![fraction, one()])), pow2(i64::from(k)))
            })
            .collect()
    };
    type Unary = fn(Complex) -> Option<usize>;
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
