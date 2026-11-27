//! Performance atlas for hyperlimit: exact geometric predicates swept across
//! distance to degeneracy, coordinate magnitude, coordinate bit length and
//! input size, for every scalar representation the predicates accept.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperlimit --all-features --example perf_atlas -- --out atlas.json`.
//!
//! Inputs are built on an integer lattice where the predicate's determinant
//! (or the relevant signed distance) is known exactly, so a configuration's
//! relative distance to degeneracy is about `2^-k`. The last point of every
//! degeneracy axis (`k = 112`) is an exactly degenerate configuration. The
//! lattice is then mapped to a representation ("kind") by a similarity, which
//! keeps every predicate sign and the relative degeneracy unchanged:
//!
//! * `f64`: lattice values scaled by a power of two, imported with
//!   `Real::try_from(f64)` (exact dyadic values with an f64 view).
//! * `integer`: the lattice integers themselves.
//! * `rational`: scaled by an odd 24-bit ratio and translated by per-axis
//!   rationals with odd denominators.
//! * `algebraic (√2, √3)`: `x √2 + t √3`; hyperreal cancels the shared
//!   radicals structurally, so cost is flat in `k`.
//! * `transcendental (π, e)`: `x π + t e`; near-degenerate inputs are settled
//!   by refinement whose cost grows with `k`, exact degeneracy structurally.
//! * `trig (cos, sin)` (in-circle/in-sphere only): points placed by cosines
//!   and sines of rational angles, so exact degeneracy hides
//!   `cos^2 + sin^2 = 1`; the `strict` variant runs under
//!   `PredicatePolicy::STRICT` and is expected to refuse that case.
//!
//! Symbolic kinds are rebuilt from rationals inside every timed call, so no
//! approximation cached by an earlier call is reused; exact kinds are built
//! once. Every kind except `strict` uses `PredicatePolicy::APPROXIMATE_512`.
//! A call that returns `PredicateOutcome::Unknown` (or `None`) is flagged as
//! an error; every flagged point in a report is a genuine refusal.
use std::cell::OnceCell;
use std::sync::OnceLock;

use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, Real, affine_independent_d, certified_interval_sign,
    classify_aabb3_intersection, classify_ball_sign, classify_circle_line2,
    classify_circle_segment2, classify_halfspace_feasibility3, classify_plane_aabb3,
    classify_point_aabb3, classify_point_convex_planes3, classify_point_convex_polygon2,
    classify_point_line_with_orientation, classify_point_oriented_plane_with_evidence,
    classify_point_plane, classify_point_ring_even_odd, classify_point_segment,
    classify_point_sphere3, classify_point_tetrahedron, classify_point_triangle,
    classify_point_triangle3, classify_ray_triangle3_intersection, classify_real_closed_interval,
    classify_real_sign, classify_segment_intersection, classify_segment_triangle3_intersection,
    classify_segment3_intersection, classify_sphere3_intersection, classify_triangle_triangle3,
    compare_point_plane_distance_squared, compare_point_segment3_distance_squared,
    compare_point2_distance_squared, compare_point3_distance_squared, compare_reals,
    construct_line_intersection_point_with_policy, incircle2, incircle2_evidence,
    incircle2_with_evidence, insphere_d, insphere3, insphere3_evidence, insphere3_with_evidence,
    intersect_segment_with_plane, intersect_three_planes, line2_orientation, orient_d, orient2,
    orient2_batch, orient3, oriented_plane3_evidence, reciprocal_real, ring_area_sign,
    ring_convexity, support_dop3_from_points,
};
use hyperlimit::{
    SegmentPlaneRelation, classify_homogeneous_point_plane, classify_point_segment3,
    classify_triangle3_degeneracy, compare_point3_lexicographic, insphere3_batch,
};
#[cfg(feature = "parallel")]
use hyperlimit::{insphere3_batch_parallel, orient2_batch_parallel};
use hyperreal::Rational;
use num::bigint::{BigInt, BigUint};
use num::{Integer, One, Signed, ToPrimitive, Zero};
use perf_atlas::{Atlas, Rng, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

type Int = BigInt;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

/// Degeneracy-axis value that stands for an exactly degenerate input.
const DEG: f64 = 112.0;
/// Deepest finite degeneracy level swept.
const K_MAX: u32 = 96;

const DEGENERACY: &str = "distance to degeneracy k (≈2^-k relative; 112 = exactly degenerate)";

/// `0, 4, ..., min(kmax, 96)`, then the exactly degenerate point.
fn degeneracy_axis(kmax: u32) -> Vec<f64> {
    (0..=kmax.min(K_MAX))
        .step_by(4)
        .map(f64::from)
        .chain([DEG])
        .collect()
}

fn level(k: f64) -> Option<u32> {
    (k < DEG).then_some(k as u32)
}

// ---------------------------------------------------------------------------
// Big-integer helpers.

fn big(rng: &mut Rng, bits: u32) -> Int {
    Int::from(BigUint::from_slice(&rng.limbs(bits)))
}

fn signed(rng: &mut Rng, bits: u32) -> Int {
    let n = big(rng, bits);
    if rng.coin() { -n } else { n }
}

fn odd(rng: &mut Rng, bits: u32) -> Int {
    big(rng, bits) | Int::one()
}

fn pow2(e: u32) -> Int {
    Int::one() << e
}

fn sign_of(n: &Int) -> Int {
    if n.is_negative() {
        -Int::one()
    } else {
        Int::one()
    }
}

fn small_primes() -> &'static [u32] {
    static PRIMES: OnceLock<Vec<u32>> = OnceLock::new();
    PRIMES.get_or_init(|| {
        (2u32..2000)
            .filter(|&n| (2..n).take_while(|d| d * d <= n).all(|d| n % d != 0))
            .collect()
    })
}

/// Miller-Rabin with fixed bases; deterministic far beyond the sizes used here
/// for practical purposes.
fn is_prime(n: &Int) -> bool {
    if *n < Int::from(2) {
        return false;
    }
    for &p in small_primes() {
        let p = Int::from(p);
        if *n == p {
            return true;
        }
        if (n % &p).is_zero() {
            return false;
        }
    }
    let n1: Int = n - Int::one();
    let s = n1.trailing_zeros().expect("n > 1");
    let d = &n1 >> s;
    'bases: for a in [2u32, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37] {
        let mut x = Int::from(a).modpow(&d, n);
        if x.is_one() || x == n1 {
            continue;
        }
        for _ in 1..s {
            x = &x * &x % n;
            if x == n1 {
                continue 'bases;
            }
        }
        return false;
    }
    true
}

/// `x^2 + y^2 = p` for a prime `p ≡ 1 (mod 4)` (Hermite-Serret / Cornacchia).
fn two_squares(p: &Int) -> (Int, Int) {
    let e = (p - 1) >> 2u32;
    let minus_one = p - 1;
    let mut c = Int::from(2);
    let mut r = loop {
        let r = c.modpow(&e, p);
        if &r * &r % p == minus_one {
            break r;
        }
        c += 1;
    };
    if &r * 2 > *p {
        r = p - r;
    }
    let (mut a, mut b) = (p.clone(), r);
    while &b * &b > *p {
        let t = &a % &b;
        a = b;
        b = t;
    }
    let y = (p - &b * &b).sqrt();
    assert_eq!(&b * &b + &y * &y, *p, "two-squares decomposition");
    (b, y)
}

/// `x^2 + y^2 + z^2 = n` for `n ≡ 1, 2 (mod 4)`.
fn three_squares(n: &Int, rng: &mut Rng) -> [Int; 3] {
    let root = n.sqrt();
    let want_even_x = n.is_odd();
    loop {
        let mut x = big(rng, root.bits() as u32) % &root;
        if x.is_even() != want_even_x {
            x += 1;
        }
        let m = n - &x * &x;
        if m.is_positive() && (&m % 4u32) == Int::one() && is_prime(&m) {
            let (y, z) = two_squares(&m);
            return [x, y, z];
        }
    }
}

/// Random signs and a random coordinate order for a decomposition.
fn scatter<const D: usize>(mut v: [Int; D], rng: &mut Rng) -> [Int; D] {
    for i in (1..D).rev() {
        v.swap(i, rng.below(i as u64 + 1) as usize);
    }
    for x in v.iter_mut() {
        if rng.coin() {
            *x = -x.clone();
        }
    }
    v
}

fn add<const D: usize>(a: &[Int; D], b: &[Int; D]) -> [Int; D] {
    std::array::from_fn(|i| &a[i] + &b[i])
}

fn sub<const D: usize>(a: &[Int; D], b: &[Int; D]) -> [Int; D] {
    std::array::from_fn(|i| &a[i] - &b[i])
}

fn scale<const D: usize>(a: &[Int; D], s: &Int) -> [Int; D] {
    std::array::from_fn(|i| &a[i] * s)
}

fn dot<const D: usize>(a: &[Int; D], b: &[Int; D]) -> Int {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn cross(a: &[Int; 3], b: &[Int; 3]) -> [Int; 3] {
    [
        &a[1] * &b[2] - &a[2] * &b[1],
        &a[2] * &b[0] - &a[0] * &b[2],
        &a[0] * &b[1] - &a[1] * &b[0],
    ]
}

fn rand_vec<const D: usize>(rng: &mut Rng, bits: u32) -> [Int; D] {
    std::array::from_fn(|_| signed(rng, bits))
}

/// Delta of about `m * 2^-k` with a random sign, never zero.
fn relative_delta(m: &Int, k: u32, rng: &mut Rng) -> Int {
    let bits = m.bits() as u32;
    let magnitude = if k + 1 < bits {
        big(rng, bits - 1 - k)
    } else {
        Int::one()
    };
    if rng.coin() { -magnitude } else { magnitude }
}

// ---------------------------------------------------------------------------
// Lattice configurations with controlled degeneracy.

/// Integer points on the circle `x^2 + y^2 = 65^2`.
fn circle_65() -> &'static [[i64; 2]] {
    static POINTS: OnceLock<Vec<[i64; 2]>> = OnceLock::new();
    POINTS.get_or_init(|| {
        let mut out = Vec::new();
        for x in -65i64..=65 {
            for y in -65i64..=65 {
                if x * x + y * y == 65 * 65 {
                    out.push([x, y]);
                }
            }
        }
        out
    })
}

/// Integer points on the sphere `x^2 + y^2 + z^2 = 9^2`.
fn sphere_9() -> &'static [[i64; 3]] {
    static POINTS: OnceLock<Vec<[i64; 3]>> = OnceLock::new();
    POINTS.get_or_init(|| {
        let mut out = Vec::new();
        for x in -9i64..=9 {
            for y in -9i64..=9 {
                for z in -9i64..=9 {
                    if x * x + y * y + z * z == 81 {
                        out.push([x, y, z]);
                    }
                }
            }
        }
        out
    })
}

fn ints<const D: usize>(v: [i64; D]) -> [Int; D] {
    v.map(Int::from)
}

/// Distinct random indices below `n`.
fn distinct(rng: &mut Rng, n: usize, count: usize) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    while out.len() < count {
        let i = rng.below(n as u64) as usize;
        if !out.contains(&i) {
            out.push(i);
        }
    }
    out
}

/// `a, b, c` with `orient2(a, b, c) = s`, `|s| ≈ 2^(2L-k)`, coordinates of
/// about `L` bits and `c` projecting between `a` and `b`. Returns `[a, b, c]`
/// and `u = b - a`.
fn near_collinear2(rng: &mut Rng, l: u32, k: Option<u32>) -> ([[Int; 2]; 3], [Int; 2]) {
    let a: [Int; 2] = rand_vec(rng, l);
    let Some(k) = k else {
        let p: [Int; 2] = rand_vec(rng, l - 12);
        let (n, m) = (big(rng, 12), big(rng, 11));
        let u = scale(&p, &n);
        let v = scale(&p, &m);
        return ([a.clone(), add(&a, &u), add(&a, &v)], u);
    };
    assert!(k <= 2 * l, "orient2 lattice supports k <= 2L");
    let (ux, uy) = loop {
        let ux = big(rng, l);
        let uy = signed(rng, l);
        if ux.gcd(&uy).is_one() {
            break (ux, uy);
        }
    };
    let s = if rng.coin() {
        pow2(2 * l - k)
    } else {
        -pow2(2 * l - k)
    };
    // ux*vy - uy*vx = s  <=>  vx ≡ -s / uy (mod ux).
    let inverse = uy.extended_gcd(&ux).x.mod_floor(&ux);
    let vx = (-(&s) * inverse).mod_floor(&ux);
    let vy = (&s + &uy * &vx) / &ux;
    debug_assert_eq!(&ux * &vy - &uy * &vx, s);
    let u = [ux, uy];
    let v = [vx, vy];
    ([a.clone(), add(&a, &u), add(&a, &v)], u)
}

#[derive(Clone, Copy)]
enum Coplanar {
    /// Fourth point anywhere near the plane.
    Generic,
    /// Fourth point near the plane, above the triangle's interior.
    Inside,
    /// Segment from the third to the fourth point crosses the line through
    /// the first two, nearly in their plane.
    Crossing,
}

struct Tetra {
    points: [[Int; 3]; 4],
    /// A far point on one side of the plane through the first three points.
    apex_offset: [Int; 3],
    /// A second in-plane direction for building far triangles.
    in_plane: [Int; 3],
}

/// `a, b, c, d` with `orient3(a, b, c, d) ≈ 2^(3L-k)` (times 16 for the
/// inside/crossing variants), mixed by a random unimodular matrix so no
/// coordinate structure survives.
fn near_coplanar3(rng: &mut Rng, l: u32, k: Option<u32>, shape: Coplanar) -> Tetra {
    let (delta, s) = match k {
        None => (Int::zero(), pow2(l)),
        Some(k) => {
            assert!(k <= 3 * l, "orient3 lattice supports k <= 3L");
            let t = 3 * l - k;
            let d = t.min(l);
            let delta = if rng.coin() { pow2(d) } else { -pow2(d) };
            (delta, pow2(t - d))
        }
    };
    // (ux, uy), (vx, vy) with ux*vy - uy*vx = s, from the 2D construction.
    let (ux, uy) = loop {
        let ux = big(rng, l);
        let uy = signed(rng, l);
        if ux.gcd(&uy).is_one() {
            break (ux, uy);
        }
    };
    let inverse = uy.extended_gcd(&ux).x.mod_floor(&ux);
    let vx = (-(&s) * inverse).mod_floor(&ux);
    let vy = (&s + &uy * &vx) / &ux;
    let u0 = [ux, uy, signed(rng, l)];
    let v0 = [vx, vy, signed(rng, l)];
    let lift = [Int::zero(), Int::zero(), delta];
    let (u, v, w) = match shape {
        Coplanar::Generic => {
            let g = Int::from(rng.range(1, 2));
            let h = Int::from([-2, -1, 1, 2][rng.below(4) as usize]);
            let w = add(&add(&scale(&u0, &g), &scale(&v0, &h)), &lift);
            (u0.clone(), v0.clone(), w)
        }
        Coplanar::Inside => {
            let w = add(&add(&u0, &v0), &lift);
            (scale(&u0, &Int::from(4)), scale(&v0, &Int::from(4)), w)
        }
        Coplanar::Crossing => {
            let w = add(&sub(&scale(&u0, &Int::from(2)), &v0), &lift);
            (scale(&u0, &Int::from(4)), scale(&v0, &Int::from(4)), w)
        }
    };
    let normal = cross(&u, &v);
    let apex: [Int; 3] = std::array::from_fn(|i| sign_of(&normal[i]) * pow2(l));
    let mix = unimodular(rng);
    let m = |x: &[Int; 3]| -> [Int; 3] {
        std::array::from_fn(|i| (0..3).map(|j| &x[j] * mix[i][j]).sum())
    };
    let a: [Int; 3] = rand_vec(rng, l);
    Tetra {
        points: [a.clone(), add(&a, &m(&u)), add(&a, &m(&v)), add(&a, &m(&w))],
        apex_offset: m(&apex),
        in_plane: m(&u0),
    }
}

/// Random unit-lower times unit-upper matrix with entries in {-1, 0, 1}.
fn unimodular(rng: &mut Rng) -> [[i64; 3]; 3] {
    let mut lower = [[0i64; 3]; 3];
    let mut upper = [[0i64; 3]; 3];
    for i in 0..3 {
        lower[i][i] = 1;
        upper[i][i] = 1;
        for j in 0..i {
            lower[i][j] = rng.range(-1, 1);
            upper[j][i] = rng.range(-1, 1);
        }
    }
    std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|t| lower[i][t] * upper[t][j]).sum())
    })
}

struct Circle {
    center: [Int; 2],
    /// Three points exactly on the circle, then the query point.
    points: [[Int; 2]; 4],
    radius_squared: Int,
}

/// Three lattice points on a circle with `|query - center|^2 = R^2 + Δ`,
/// `Δ ≈ R^2 2^-k` (exactly zero when degenerate). The query point is found
/// by writing a prime as a sum of two squares.
fn near_cocircular2(rng: &mut Rng, k: Option<u32>) -> Circle {
    let center: [Int; 2] = rand_vec(rng, 51);
    let (zx, zy) = (signed(rng, 44), signed(rng, 44));
    let rotate = |v: [i64; 2]| -> [Int; 2] {
        let (vx, vy) = (Int::from(v[0]), Int::from(v[1]));
        [&zx * &vx - &zy * &vy, &zx * &vy + &zy * &vx]
    };
    let set = circle_65();
    let idx = distinct(rng, set.len(), 4);
    let m = (&zx * &zx + &zy * &zy) * 4225;
    let on = |i: usize| add(&center, &rotate(set[idx[i]]));
    let query = match k {
        None => on(3),
        Some(k) => {
            let mut delta = relative_delta(&m, k, rng);
            let step = sign_of(&delta);
            loop {
                let n = &m + &delta;
                if (&n % 4u32) == Int::one() && is_prime(&n) {
                    let (x, y) = two_squares(&n);
                    break add(&center, &scatter([x, y], rng));
                }
                delta += &step;
            }
        }
    };
    Circle {
        points: [on(0), on(1), on(2), query],
        center,
        radius_squared: m,
    }
}

struct Sphere {
    center: [Int; 3],
    /// Four points exactly on the sphere, then the query point.
    points: [[Int; 3]; 5],
    radius_squared: Int,
    radius: Int,
}

/// Four lattice points on a sphere (images of `|v| = 9` points under a random
/// integer quaternion rotation) and a query point with
/// `|query - center|^2 = R^2 + Δ`, `Δ ≈ R^2 2^-k`.
fn near_cospherical3(rng: &mut Rng, k: Option<u32>) -> Sphere {
    let center: [Int; 3] = rand_vec(rng, 50);
    let [a, b, c, d]: [Int; 4] = rand_vec(rng, 22);
    let norm = &a * &a + &b * &b + &c * &c + &d * &d;
    let rot = [
        [
            &a * &a + &b * &b - &c * &c - &d * &d,
            (&b * &c - &a * &d) * 2,
            (&b * &d + &a * &c) * 2,
        ],
        [
            (&b * &c + &a * &d) * 2,
            &a * &a - &b * &b + &c * &c - &d * &d,
            (&c * &d - &a * &b) * 2,
        ],
        [
            (&b * &d - &a * &c) * 2,
            (&c * &d + &a * &b) * 2,
            &a * &a - &b * &b - &c * &c + &d * &d,
        ],
    ];
    let rotate = |v: [i64; 3]| -> [Int; 3] {
        std::array::from_fn(|i| (0..3).map(|j| &rot[i][j] * v[j]).sum())
    };
    let set = sphere_9();
    let idx = loop {
        let idx = distinct(rng, set.len(), 5);
        let p: Vec<[Int; 3]> = idx.iter().map(|&i| ints(set[i])).collect();
        let det = dot(
            &cross(&sub(&p[1], &p[0]), &sub(&p[2], &p[0])),
            &sub(&p[3], &p[0]),
        );
        if !det.is_zero() {
            break idx;
        }
    };
    let radius = &norm * 9;
    let m = &radius * &radius;
    let on = |i: usize| add(&center, &rotate(set[idx[i]]));
    let query = match k {
        None => on(4),
        Some(k) => {
            let mut delta = relative_delta(&m, k, rng);
            let step = sign_of(&delta);
            loop {
                let n = &m + &delta;
                let r = (&n % 4u32).to_u32().expect("small");
                if r == 1 || r == 2 {
                    break add(&center, &scatter(three_squares(&n, rng), rng));
                }
                delta += &step;
            }
        }
    };
    Sphere {
        points: [on(0), on(1), on(2), on(3), query],
        center,
        radius_squared: m,
        radius,
    }
}

/// Explicit plane `n . x + off = 0` with 24-bit normal and a point `p` of
/// `l` bits at signed value `n . p + off = t`, `|t| ≈ 2^(l+24-k)`.
fn near_plane(rng: &mut Rng, l: u32, k: Option<u32>) -> ([Int; 3], Int, [Int; 3], Int) {
    let normal: [Int; 3] = rand_vec(rng, 24);
    let point: [Int; 3] = rand_vec(rng, l);
    let t = match k {
        None => Int::zero(),
        Some(k) => {
            let bits = l + 24 - k.min(l + 23);
            if rng.coin() { pow2(bits) } else { -pow2(bits) }
        }
    };
    let offset = &t - dot(&normal, &point);
    (normal, offset, point, t)
}

// ---------------------------------------------------------------------------
// Representations.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    F64,
    Integer,
    Rational,
    Algebraic,
    Transcendental,
    Trig,
    TrigStrict,
}

impl Kind {
    fn label(self) -> &'static str {
        match self {
            Kind::F64 => "f64",
            Kind::Integer => "integer",
            Kind::Rational => "rational",
            Kind::Algebraic => "algebraic (√2, √3)",
            Kind::Transcendental => "transcendental (π, e)",
            Kind::Trig => "trig (cos, sin)",
            Kind::TrigStrict => "trig (cos, sin), strict",
        }
    }

    fn policy(self) -> PredicatePolicy {
        if self == Kind::TrigStrict {
            STRICT
        } else {
            APPROX
        }
    }
}

const DET_KINDS: [Kind; 5] = [
    Kind::F64,
    Kind::Integer,
    Kind::Rational,
    Kind::Algebraic,
    Kind::Transcendental,
];
const KINDS: [Kind; 4] = [
    Kind::F64,
    Kind::Rational,
    Kind::Algebraic,
    Kind::Transcendental,
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Atom {
    Sqrt2,
    Sqrt3,
    Pi,
    E,
    PiSquared,
}

/// Irrational constants, built once per timed call on first use.
#[derive(Default)]
struct Atoms {
    sqrt2: OnceCell<Real>,
    sqrt3: OnceCell<Real>,
    pi: OnceCell<Real>,
    e: OnceCell<Real>,
    pi_squared: OnceCell<Real>,
}

impl Atoms {
    fn get(&self, atom: Atom) -> &Real {
        match atom {
            Atom::Sqrt2 => self
                .sqrt2
                .get_or_init(|| Real::from(2).sqrt().expect("sqrt 2")),
            Atom::Sqrt3 => self
                .sqrt3
                .get_or_init(|| Real::from(3).sqrt().expect("sqrt 3")),
            Atom::Pi => self.pi.get_or_init(Real::pi),
            Atom::E => self.e.get_or_init(Real::e),
            Atom::PiSquared => self.pi_squared.get_or_init(|| {
                let pi = self.get(Atom::Pi);
                pi * pi
            }),
        }
    }
}

/// One scalar of an input: ready-made, or an expression built inside the
/// timed call.
#[derive(Clone)]
enum Term {
    Ready(Real),
    /// `sum c_i atom_i`.
    Sum(Vec<(Rational, Atom)>),
    /// `offset + radius * prod (cos|sin)(angle)`; `true` selects sine.
    Trig {
        offset: Rational,
        radius: Rational,
        factors: Vec<(Rational, bool)>,
    },
}

impl Term {
    fn real(&self, atoms: &Atoms) -> Real {
        match self {
            Term::Ready(r) => r.clone(),
            Term::Sum(parts) => parts
                .iter()
                .filter(|(c, _)| !c.is_zero())
                .map(|(c, atom)| &Real::new(c.clone()) * atoms.get(*atom))
                .reduce(|a, b| &a + &b)
                .unwrap_or_else(Real::zero),
            Term::Trig {
                offset,
                radius,
                factors,
            } => {
                let mut value = Real::new(radius.clone());
                for (angle, sine) in factors {
                    let angle = Real::new(angle.clone());
                    value = &value * &if *sine { angle.sin() } else { angle.cos() };
                }
                &Real::new(offset.clone()) + &value
            }
        }
    }
}

fn dyadic(n: &Int, exponent: i64) -> Rational {
    if exponent >= 0 {
        Rational::from_bigint(n << exponent as u64)
    } else {
        Rational::from_bigint_fraction(n.clone(), BigUint::one() << (-exponent) as u64)
            .expect("nonzero denominator")
    }
}

fn ratio(n: &Int, d: &Int) -> Rational {
    Rational::from_bigint_fraction(n.clone(), d.magnitude().clone()).expect("nonzero denominator")
}

fn int_real(n: &Int) -> Real {
    Real::new(Rational::from_bigint(n.clone()))
}

fn ldexp(mut x: f64, mut e: i64) -> f64 {
    while e > 500 {
        x *= 2f64.powi(500);
        e -= 500;
    }
    while e < -500 {
        x *= 2f64.powi(-500);
        e += 500;
    }
    x * 2f64.powi(e as i32)
}

/// Maps lattice integers to one representation by a similarity
/// `x -> s (P x + T) + t`: `P`, `T` widen coordinates to a target bit length,
/// `s` and `t` depend on the kind (`s = 2^(e-b)` times 1, an odd ratio, `√2`
/// or `π`; `t` zero, rational, `√3`-multiple or `e`-multiple).
struct Frame {
    kind: Kind,
    /// Bits of the (widened) lattice; `2^(exponent - bits)` scales values to
    /// about `2^exponent`.
    bits: u32,
    exponent: i32,
    widen: Option<(Int, [Int; 3])>,
    p: Int,
    q: Int,
    tau: [Int; 3],
    tau_den: [Int; 3],
}

impl Frame {
    /// Lattice values of about `base` bits mapped to magnitude `2^exponent`.
    fn new(kind: Kind, base: u32, exponent: i32, rng: &mut Rng) -> Self {
        Self {
            kind,
            bits: base,
            exponent,
            widen: None,
            p: odd(rng, 24),
            q: odd(rng, 24),
            tau: rand_vec(rng, base),
            tau_den: std::array::from_fn(|_| odd(rng, 16)),
        }
    }

    /// Lattice values of about `base` bits widened to about `bits` bits.
    fn widened(kind: Kind, base: u32, bits: u32, rng: &mut Rng) -> Self {
        assert!(kind != Kind::F64, "f64 coordinates cannot be widened");
        let factor = odd(rng, bits.saturating_sub(base).max(2));
        let shift = rand_vec(rng, bits);
        Self {
            widen: Some((factor, shift)),
            ..Self::new(kind, bits, 0, rng)
        }
    }

    fn lattice(&self, axis: usize, n: &Int) -> Int {
        match &self.widen {
            Some((p, t)) => p * n + &t[axis],
            None => n.clone(),
        }
    }

    fn shift(&self) -> i64 {
        i64::from(self.exponent) - i64::from(self.bits)
    }

    fn f64(&self, axis: usize, n: &Int) -> f64 {
        let n = self.lattice(axis, n);
        assert!(n.bits() <= 53, "f64 lattice value exceeds 53 bits");
        assert!(self.shift() >= -1074, "f64 lattice value underflows");
        ldexp(n.to_f64().expect("53-bit value"), self.shift())
    }

    /// The irrational scale and translation atoms of this kind.
    fn atoms(&self) -> (Atom, Atom) {
        match self.kind {
            Kind::Algebraic => (Atom::Sqrt2, Atom::Sqrt3),
            Kind::Transcendental => (Atom::Pi, Atom::E),
            _ => unreachable!("not a symbolic kind"),
        }
    }

    fn coord(&self, axis: usize, n: &Int) -> Term {
        match self.kind {
            Kind::F64 => Term::Ready(Real::try_from(self.f64(axis, n)).expect("finite")),
            Kind::Integer => Term::Ready(int_real(&self.lattice(axis, n))),
            Kind::Rational => {
                let x = self.lattice(axis, n);
                let value = &(&ratio(&(&self.p * &x), &self.q)
                    + &ratio(&self.tau[axis], &self.tau_den[axis]))
                    * &dyadic(&Int::one(), self.shift());
                Term::Ready(Real::new(value))
            }
            Kind::Algebraic | Kind::Transcendental => {
                let (scale, translate) = self.atoms();
                Term::Sum(vec![
                    (dyadic(&self.lattice(axis, n), self.shift()), scale),
                    (dyadic(&self.tau[axis], self.shift()), translate),
                ])
            }
            Kind::Trig | Kind::TrigStrict => unreachable!("trig inputs are built directly"),
        }
    }

    fn widen_factor(&self) -> Int {
        self.widen
            .as_ref()
            .map_or_else(Int::one, |(p, _)| p.clone())
    }

    /// A difference vector component (scaled, not translated).
    fn length(&self, n: &Int) -> Term {
        let n = self.widen_factor() * n;
        match self.kind {
            Kind::F64 => {
                assert!(n.bits() <= 53);
                Term::Ready(Real::try_from(ldexp(n.to_f64().unwrap(), self.shift())).unwrap())
            }
            Kind::Integer => Term::Ready(int_real(&n)),
            Kind::Rational => Term::Ready(Real::new(
                &ratio(&(&self.p * &n), &self.q) * &dyadic(&Int::one(), self.shift()),
            )),
            _ => Term::Sum(vec![(dyadic(&n, self.shift()), self.atoms().0)]),
        }
    }

    /// A squared length (scales by `s^2`).
    fn squared(&self, n: &Int) -> Term {
        let w = self.widen_factor();
        let n = &w * &w * n;
        let shift = 2 * self.shift();
        Term::Ready(Real::new(match self.kind {
            Kind::F64 => dyadic(&n, shift),
            Kind::Integer => Rational::from_bigint(n),
            Kind::Rational => {
                &ratio(&(&self.p * &self.p * &n), &(&self.q * &self.q))
                    * &dyadic(&Int::one(), shift)
            }
            Kind::Algebraic => dyadic(&(n * 2), shift),
            _ => return Term::Sum(vec![(dyadic(&n, shift), Atom::PiSquared)]),
        }))
    }

    /// A scale-free coefficient such as a plane normal component.
    fn plain(&self, n: &Int) -> Term {
        match self.kind {
            Kind::F64 => {
                assert!(n.bits() <= 53);
                Term::Ready(Real::try_from(n.to_f64().unwrap()).unwrap())
            }
            _ => Term::Ready(int_real(n)),
        }
    }

    /// Offset of the plane `normal . x + offset = 0` after the similarity.
    fn offset(&self, normal: &[Int; 3], offset: &Int) -> Term {
        let off = match &self.widen {
            Some((p, t)) => p * offset - dot(normal, t),
            None => offset.clone(),
        };
        match self.kind {
            Kind::F64 => Term::Ready(Real::new(dyadic(&off, self.shift()))),
            Kind::Integer => Term::Ready(int_real(&off)),
            Kind::Rational => {
                let mut value = ratio(&(&self.p * &off), &self.q);
                for i in 0..3 {
                    value = &value - &ratio(&(&normal[i] * &self.tau[i]), &self.tau_den[i]);
                }
                Term::Ready(Real::new(&value * &dyadic(&Int::one(), self.shift())))
            }
            _ => {
                let (scale, translate) = self.atoms();
                Term::Sum(vec![
                    (dyadic(&off, self.shift()), scale),
                    (dyadic(&-dot(normal, &self.tau), self.shift()), translate),
                ])
            }
        }
    }

    fn points<const D: usize>(&self, points: &[[Int; D]]) -> Vec<Term> {
        points
            .iter()
            .flat_map(|p| p.iter().enumerate().map(|(axis, n)| self.coord(axis, n)))
            .collect()
    }

    fn plane(&self, normal: &[Int; 3], offset: &Int) -> Vec<Term> {
        let mut out: Vec<Term> = normal.iter().map(|n| self.plain(n)).collect();
        out.push(self.offset(normal, offset));
        out
    }
}

/// Random rational angle `m / 256` with `m` in `lo..=hi`, apart from `used`.
fn angle(rng: &mut Rng, lo: i64, hi: i64, used: &mut Vec<i64>) -> Rational {
    loop {
        let m = rng.range(lo, hi);
        if used.iter().all(|u| (u - m).abs() > 8) {
            used.push(m);
            return Rational::fraction(m, 256).unwrap();
        }
    }
}

/// `1 + m 2^-(k+7)` with an 8-bit `m`: a relative bump of about `2^-k`.
fn bump(rng: &mut Rng, k: Option<u32>) -> Rational {
    match k {
        None => Rational::one(),
        Some(k) => &Rational::one() + &dyadic(&big(rng, 8), -(i64::from(k) + 7)),
    }
}

/// Three points `c + R (cos θ, sin θ)` on a circle and a query at radius
/// `R (1 + ≈2^-k)`; exact degeneracy hides `cos^2 + sin^2 = 1`.
fn trig_circle(rng: &mut Rng, k: Option<u32>) -> Vec<Term> {
    let center = [dyadic(&signed(rng, 40), -20), dyadic(&signed(rng, 40), -20)];
    let radius = &dyadic(&big(rng, 40), -40) + &Rational::one();
    let mut used = Vec::new();
    let mut out = Vec::new();
    for i in 0..4 {
        let theta = angle(rng, 0, 1600, &mut used);
        let r = if i == 3 {
            &radius * &bump(rng, k)
        } else {
            radius.clone()
        };
        for (axis, sine) in [(0, false), (1, true)] {
            out.push(Term::Trig {
                offset: center[axis].clone(),
                radius: r.clone(),
                factors: vec![(theta.clone(), sine)],
            });
        }
    }
    out
}

/// Four points `c + R (cos φ cos θ, cos φ sin θ, sin φ)` on a sphere and a
/// query at radius `R (1 + ≈2^-k)`.
fn trig_sphere(rng: &mut Rng, k: Option<u32>) -> Vec<Term> {
    let center: [Rational; 3] = std::array::from_fn(|_| dyadic(&signed(rng, 40), -20));
    let radius = &dyadic(&big(rng, 40), -40) + &Rational::one();
    let (mut used_theta, mut used_phi) = (Vec::new(), Vec::new());
    let mut out = Vec::new();
    for i in 0..5 {
        let theta = angle(rng, 0, 1600, &mut used_theta);
        let phi = angle(rng, -380, 380, &mut used_phi);
        let r = if i == 4 {
            &radius * &bump(rng, k)
        } else {
            radius.clone()
        };
        let factors = [
            vec![(phi.clone(), false), (theta.clone(), false)],
            vec![(phi.clone(), false), (theta.clone(), true)],
            vec![(phi.clone(), true)],
        ];
        for (axis, factors) in factors.into_iter().enumerate() {
            out.push(Term::Trig {
                offset: center[axis].clone(),
                radius: r.clone(),
                factors,
            });
        }
    }
    out
}

/// Points, planes and remaining scalars of one input.
struct Geo<P> {
    p: Vec<P>,
    planes: Vec<Plane3>,
    s: Vec<Real>,
}

trait FromReals: Sized {
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self;
}

impl FromReals for Point2 {
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self {
        Point2::new(it.next().unwrap(), it.next().unwrap())
    }
}

impl FromReals for Point3 {
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self {
        Point3::new(it.next().unwrap(), it.next().unwrap(), it.next().unwrap())
    }
}

fn geo<P: FromReals>(reals: Vec<Real>, points: usize, planes: usize) -> Geo<P> {
    let mut it = reals.into_iter();
    let p = (0..points).map(|_| P::take(&mut it)).collect();
    let planes = (0..planes)
        .map(|_| {
            let normal = Point3::take(&mut it);
            Plane3::new(normal, it.next().unwrap())
        })
        .collect();
    Geo {
        p,
        planes,
        s: it.collect(),
    }
}

/// An input ready for timing: exact inputs are built once, symbolic ones are
/// rebuilt from their terms in every call so no cached approximation is
/// reused.
struct Input<G> {
    policy: PredicatePolicy,
    warm: Option<G>,
    terms: Vec<Term>,
    build: fn(Vec<Real>) -> G,
}

impl<G> Input<G> {
    fn new(kind: Kind, terms: Vec<Term>, build: fn(Vec<Real>) -> G) -> Self {
        if terms.iter().any(|t| !matches!(t, Term::Ready(..))) {
            return Self {
                policy: kind.policy(),
                warm: None,
                terms,
                build,
            };
        }
        let reals = terms
            .into_iter()
            .map(|t| match t {
                Term::Ready(r) => r,
                _ => unreachable!(),
            })
            .collect();
        Self {
            policy: kind.policy(),
            warm: Some(build(reals)),
            terms: Vec::new(),
            build,
        }
    }

    fn run<R>(&self, f: impl FnOnce(&G, PredicatePolicy) -> R) -> R {
        if let Some(g) = &self.warm {
            return f(g, self.policy);
        }
        let atoms = Atoms::default();
        let g = (self.build)(self.terms.iter().map(|t| t.real(&atoms)).collect());
        f(&g, self.policy)
    }
}

type In2 = Input<Geo<Point2>>;
type In3 = Input<Geo<Point3>>;

// ---------------------------------------------------------------------------
// The four determinant predicates.

/// Lattice bits used for each determinant construction so that f64 views
/// stay exact.
const ORIENT2_BITS: u32 = 48;
const ORIENT3_BITS: u32 = 44;
const CIRCLE_BITS: u32 = 52;
const SPHERE_BITS: u32 = 52;

/// Builds lattice points at a degeneracy level (`None`: exactly degenerate).
type Lattice<const D: usize> = fn(&mut Rng, Option<u32>) -> Vec<[Int; D]>;

struct Det {
    name: &'static str,
    what: &'static str,
    base: u32,
    kmax: u32,
    make2: Option<Lattice<2>>,
    make3: Option<Lattice<3>>,
    run2: fn(&Geo<Point2>, PredicatePolicy) -> Option<hyperlimit::Sign>,
    run3: fn(&Geo<Point3>, PredicatePolicy) -> Option<hyperlimit::Sign>,
}

fn orient2_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 2]> {
    near_collinear2(rng, ORIENT2_BITS, k).0.to_vec()
}
fn orient3_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 3]> {
    near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Generic)
        .points
        .to_vec()
}
fn incircle_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 2]> {
    near_cocircular2(rng, k).points.to_vec()
}
fn insphere_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 3]> {
    near_cospherical3(rng, k).points.to_vec()
}

fn no2(_: &Geo<Point2>, _: PredicatePolicy) -> Option<hyperlimit::Sign> {
    unreachable!()
}
fn no3(_: &Geo<Point3>, _: PredicatePolicy) -> Option<hyperlimit::Sign> {
    unreachable!()
}

fn dets() -> [Det; 4] {
    [
        Det {
            name: "orient2",
            what: "Sign of the 2D orientation determinant of three points",
            base: ORIENT2_BITS,
            kmax: 2 * ORIENT2_BITS,
            make2: Some(orient2_points),
            make3: None,
            run2: |g, pol| orient2(&g.p[0], &g.p[1], &g.p[2], pol).value(),
            run3: no3,
        },
        Det {
            name: "orient3",
            what: "Sign of the 3D orientation determinant of four points",
            base: ORIENT3_BITS,
            kmax: 3 * ORIENT3_BITS,
            make2: None,
            make3: Some(orient3_points),
            run2: no2,
            run3: |g, pol| orient3(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
        },
        Det {
            name: "incircle2",
            what: "In-circle test of a point against the circle through three points",
            base: CIRCLE_BITS,
            kmax: 92,
            make2: Some(incircle_points),
            make3: None,
            run2: |g, pol| incircle2(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
            run3: no3,
        },
        Det {
            name: "insphere3",
            what: "In-sphere test of a point against the sphere through four points",
            base: SPHERE_BITS,
            kmax: 92,
            make2: None,
            make3: Some(insphere_points),
            run2: no2,
            run3: |g, pol| insphere3(&g.p[0], &g.p[1], &g.p[2], &g.p[3], &g.p[4], pol).value(),
        },
    ]
}

impl Det {
    /// Sweep one kind/frame family over an axis, building the lattice
    /// configuration at degeneracy `k(m)` in the frame `frame(m)`.
    fn sweep(
        &self,
        atlas: &mut Atlas,
        label: &str,
        axis: &[f64],
        kind: Kind,
        k: impl Fn(f64) -> Option<u32>,
        frame: impl Fn(f64, &mut Rng) -> Frame,
    ) {
        if let Some(make) = self.make2 {
            let run = self.run2;
            let build = self.build2();
            atlas.sweep(
                label,
                axis,
                |m, rng| {
                    let pts = make(rng, k(m));
                    In2::new(kind, frame(m, rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        } else if let Some(make) = self.make3 {
            let run = self.run3;
            let build = self.build3();
            atlas.sweep(
                label,
                axis,
                |m, rng| {
                    let pts = make(rng, k(m));
                    In3::new(kind, frame(m, rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        }
    }

    /// Builder for this predicate's 2D points (3 for orient2, 4 for incircle2).
    fn build2(&self) -> fn(Vec<Real>) -> Geo<Point2> {
        if self.name == "orient2" {
            geo2_3
        } else {
            geo2_4
        }
    }

    /// Builder for this predicate's 3D points (4 for orient3, 5 for insphere3).
    fn build3(&self) -> fn(Vec<Real>) -> Geo<Point3> {
        if self.name == "orient3" {
            geo3_4
        } else {
            geo3_5
        }
    }

    fn grid(
        &self,
        atlas: &mut Atlas,
        label: &str,
        kind: Kind,
        b: (&str, &[f64]),
        ks: &[f64],
        frame: impl Fn(f64, &mut Rng) -> Frame,
    ) {
        if let Some(make) = self.make2 {
            let run = self.run2;
            let build = self.build2();
            atlas.grid(
                label,
                (DEGENERACY, ks),
                b,
                |k, m, rng| {
                    let pts = make(rng, level(k));
                    In2::new(kind, frame(m, rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        } else if let Some(make) = self.make3 {
            let run = self.run3;
            let build = self.build3();
            atlas.grid(
                label,
                (DEGENERACY, ks),
                b,
                |k, m, rng| {
                    let pts = make(rng, level(k));
                    In3::new(kind, frame(m, rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        }
    }

    fn fuzz(
        &self,
        atlas: &mut Atlas,
        label: &str,
        axis: &[f64],
        kind: Kind,
        frame: impl Fn(&mut Rng) -> Frame,
    ) {
        let kmax = self.kmax.min(K_MAX);
        let k = move |m: f64, rng: &mut Rng| -> Option<u32> {
            if m >= DEG {
                None
            } else {
                Some((m as u32 + rng.below(8) as u32).min(kmax))
            }
        };
        if let Some(make) = self.make2 {
            let run = self.run2;
            let build = self.build2();
            atlas.fuzz(
                label,
                axis,
                |m, rng| {
                    let level = k(m, rng);
                    let pts = make(rng, level);
                    In2::new(kind, frame(rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        } else if let Some(make) = self.make3 {
            let run = self.run3;
            let build = self.build3();
            atlas.fuzz(
                label,
                axis,
                |m, rng| {
                    let level = k(m, rng);
                    let pts = make(rng, level);
                    In3::new(kind, frame(rng).points(&pts), build)
                },
                |input| input.run(run),
            );
        }
    }

    /// Shewchuk's adaptive f64 predicate (`robust` crate) on the same f64
    /// inputs, for reference.
    fn reference(&self, atlas: &mut Atlas, axis: &[f64]) {
        let base = self.base;
        let frame = move |rng: &mut Rng| Frame::new(Kind::F64, base, 0, rng);
        let label = "robust f64 (reference)";
        match self.name {
            "orient2" => atlas.sweep(
                label,
                axis,
                |m, rng| raw2(&frame(rng), &orient2_points(rng, level(m))),
                |p| robust::orient2d(c2(p[0]), c2(p[1]), c2(p[2])),
            ),
            "orient3" => atlas.sweep(
                label,
                axis,
                |m, rng| raw3(&frame(rng), &orient3_points(rng, level(m))),
                |p| robust::orient3d(c3(p[0]), c3(p[1]), c3(p[2]), c3(p[3])),
            ),
            "incircle2" => atlas.sweep(
                label,
                axis,
                |m, rng| raw2(&frame(rng), &incircle_points(rng, level(m))),
                |p| robust::incircle(c2(p[0]), c2(p[1]), c2(p[2]), c2(p[3])),
            ),
            "insphere3" => atlas.sweep(
                label,
                axis,
                |m, rng| raw3(&frame(rng), &insphere_points(rng, level(m))),
                |p| robust::insphere(c3(p[0]), c3(p[1]), c3(p[2]), c3(p[3]), c3(p[4])),
            ),
            _ => unreachable!(),
        }
    }

    /// Points on a circle or sphere given by cosines and sines, so exact
    /// degeneracy hides `cos^2 + sin^2 = 1`.
    fn trig(&self, atlas: &mut Atlas, axis: &[f64]) {
        for kind in [Kind::Trig, Kind::TrigStrict] {
            match self.name {
                "incircle2" => atlas.sweep(
                    kind.label(),
                    axis,
                    |m, rng| In2::new(kind, trig_circle(rng, level(m)), geo2_4),
                    |input| input.run(self.run2),
                ),
                "insphere3" => atlas.sweep(
                    kind.label(),
                    axis,
                    |m, rng| In3::new(kind, trig_sphere(rng, level(m)), geo3_5),
                    |input| input.run(self.run3),
                ),
                _ => {}
            }
        }
    }

    /// The predicate with evidence prepared from the fixed points outside the
    /// timer, next to the direct call on the same inputs.
    fn prepared(&self, atlas: &mut Atlas, axis: &[f64]) {
        for kind in [Kind::F64, Kind::Rational] {
            let base = self.base;
            let frame = move |rng: &mut Rng| Frame::new(kind, base, 0, rng);
            self.sweep(
                atlas,
                &format!("{} direct", kind.label()),
                axis,
                kind,
                level,
                |_, rng| Frame::new(kind, base, 0, rng),
            );
            let label = format!("{} prepared", kind.label());
            match self.name {
                "orient2" => atlas.sweep(
                    &label,
                    axis,
                    |m, rng| {
                        let pts = orient2_points(rng, level(m));
                        Input::new(kind, frame(rng).points(&pts), |v| {
                            let g = geo::<Point2>(v, 3, 0);
                            let o = line2_orientation(&g.p[0], &g.p[1]);
                            (g, o)
                        })
                    },
                    |input| {
                        input.run(|(g, o), pol| {
                            classify_point_line_with_orientation(&g.p[0], &g.p[1], &g.p[2], o, pol)
                                .value()
                        })
                    },
                ),
                "orient3" => atlas.sweep(
                    &label,
                    axis,
                    |m, rng| {
                        let pts = orient3_points(rng, level(m));
                        Input::new(kind, frame(rng).points(&pts), |v| {
                            let g = geo::<Point3>(v, 4, 0);
                            let e = oriented_plane3_evidence(&g.p[0], &g.p[1], &g.p[2]);
                            (g, e)
                        })
                    },
                    |input| {
                        input.run(|(g, e), pol| {
                            classify_point_oriented_plane_with_evidence(&g.p[3], e, pol).value()
                        })
                    },
                ),
                "incircle2" => atlas.sweep(
                    &label,
                    axis,
                    |m, rng| {
                        let pts = incircle_points(rng, level(m));
                        Input::new(kind, frame(rng).points(&pts), |v| {
                            let g = geo::<Point2>(v, 4, 0);
                            let e = incircle2_evidence(&g.p[0], &g.p[1], &g.p[2]);
                            (g, e)
                        })
                    },
                    |input| {
                        input.run(|(g, e), pol| {
                            incircle2_with_evidence(&g.p[0], &g.p[1], &g.p[2], &g.p[3], e, pol)
                                .value()
                        })
                    },
                ),
                "insphere3" => atlas.sweep(
                    &label,
                    axis,
                    |m, rng| {
                        let pts = insphere_points(rng, level(m));
                        Input::new(kind, frame(rng).points(&pts), |v| {
                            let g = geo::<Point3>(v, 5, 0);
                            let e = insphere3_evidence(&g.p[0], &g.p[1], &g.p[2], &g.p[3]);
                            (g, e)
                        })
                    },
                    |input| {
                        input.run(|(g, e), pol| {
                            let p = &g.p;
                            insphere3_with_evidence(&p[0], &p[1], &p[2], &p[3], &p[4], e, pol)
                                .value()
                        })
                    },
                ),
                _ => unreachable!(),
            }
        }
    }
}

fn geo2_3(v: Vec<Real>) -> Geo<Point2> {
    geo(v, 3, 0)
}
fn geo2_4(v: Vec<Real>) -> Geo<Point2> {
    geo(v, 4, 0)
}
fn geo3_4(v: Vec<Real>) -> Geo<Point3> {
    geo(v, 4, 0)
}
fn geo3_5(v: Vec<Real>) -> Geo<Point3> {
    geo(v, 5, 0)
}

fn raw2(frame: &Frame, pts: &[[Int; 2]]) -> Vec<[f64; 2]> {
    pts.iter()
        .map(|p| [frame.f64(0, &p[0]), frame.f64(1, &p[1])])
        .collect()
}
fn raw3(frame: &Frame, pts: &[[Int; 3]]) -> Vec<[f64; 3]> {
    pts.iter()
        .map(|p| {
            [
                frame.f64(0, &p[0]),
                frame.f64(1, &p[1]),
                frame.f64(2, &p[2]),
            ]
        })
        .collect()
}
fn c2(p: [f64; 2]) -> robust::Coord<f64> {
    robust::Coord { x: p[0], y: p[1] }
}
fn c3(p: [f64; 3]) -> robust::Coord3D<f64> {
    robust::Coord3D {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}

fn determinant_predicates(atlas: &mut Atlas) {
    let exponents: Vec<f64> = (-1000..=960).step_by(40).map(f64::from).collect();
    let grid_exponents: Vec<f64> = (-1000..=1000).step_by(125).map(f64::from).collect();
    let bits: Vec<f64> = [
        64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096,
    ]
    .map(f64::from)
    .to_vec();
    let grid_bits: Vec<f64> = powers_of_two(6, 12);
    let fuzz_axis: Vec<f64> = (0..=88).step_by(8).map(f64::from).chain([DEG]).collect();

    for det in dets() {
        let base = det.base;
        let axis = degeneracy_axis(det.kmax);
        let trig = if matches!(det.name, "incircle2" | "insphere3") {
            " Trig kinds place points by cos/sin, so an exactly degenerate input hides cos² + sin² = 1: STRICT refuses it (flagged), APPROXIMATE_512 settles it with an approximate certificate."
        } else {
            ""
        };
        atlas.operation(
            "determinants",
            &format!("{} by degeneracy", det.name),
            DEGENERACY,
            &format!(
                "{} with coordinates near 1, as the configuration approaches degeneracy. Filter failure shows as a cliff near k ≈ 50; symbolic kinds rebuild their coordinates in every call (a few µs).{trig}",
                det.what
            ),
        );
        for kind in DET_KINDS {
            det.sweep(atlas, kind.label(), &axis, kind, level, |_, rng| {
                Frame::new(kind, base, 0, rng)
            });
        }
        det.trig(atlas, &axis);
        det.reference(atlas, &axis);
        det.fuzz(atlas, "f64", &fuzz_axis, Kind::F64, |rng| {
            let e = rng.range(-300, 300) as i32;
            Frame::new(Kind::F64, base, e, rng)
        });
        det.fuzz(atlas, "rational", &fuzz_axis, Kind::Rational, |rng| {
            let bits = rng.range(64, 1024) as u32;
            Frame::widened(Kind::Rational, base, bits, rng)
        });
        let ks: Vec<f64> = (0..=det.kmax.min(96))
            .step_by(8)
            .map(f64::from)
            .chain([DEG])
            .collect();
        det.grid(
            atlas,
            "f64",
            Kind::F64,
            ("coordinate exponent (x ≈ 2^e)", &grid_exponents),
            &ks,
            |e, rng| Frame::new(Kind::F64, base, e as i32, rng),
        );
        for kind in [Kind::Integer, Kind::Transcendental] {
            let grid_bits = if kind == Kind::Integer {
                &grid_bits[..]
            } else {
                &grid_bits[..4]
            };
            det.grid(
                atlas,
                kind.label(),
                kind,
                ("coordinate bits", grid_bits),
                &ks,
                |b, rng| Frame::widened(kind, base, b as u32, rng),
            );
        }

        atlas.operation(
            "determinants",
            &format!("{} prepared evidence", det.name),
            DEGENERACY,
            &format!(
                "{}: direct call against the variant that reuses evidence prepared from the fixed points outside the timer.",
                det.what
            ),
        );
        det.prepared(atlas, &axis);

        atlas.operation(
            "determinants",
            &format!("{} by coordinate exponent", det.name),
            "coordinate exponent (x ≈ 2^e)",
            &format!(
                "{} with coordinates scaled by 2^e at fixed distances to degeneracy; products of coordinates leave the f64 range at the extremes.",
                det.what
            ),
        );
        let series: [(&str, Kind, Option<u32>); 6] = [
            ("f64, generic (k=0)", Kind::F64, Some(0)),
            ("f64, near (k=48)", Kind::F64, Some(48)),
            ("f64, very near (k=88)", Kind::F64, Some(88)),
            ("f64, degenerate", Kind::F64, None),
            ("rational, near (k=48)", Kind::Rational, Some(48)),
            (
                "transcendental, near (k=48)",
                Kind::Transcendental,
                Some(48),
            ),
        ];
        for (label, kind, k) in series {
            det.sweep(
                atlas,
                label,
                &exponents,
                kind,
                |_| k,
                |e, rng| Frame::new(kind, base, e as i32, rng),
            );
        }

        atlas.operation(
            "determinants",
            &format!("{} by coordinate bits", det.name),
            "coordinate bits",
            &format!(
                "{} for exact inputs whose coordinates are widened to about b bits by an integer similarity (relative degeneracy unchanged).",
                det.what
            ),
        );
        let series: [(&str, Kind, Option<u32>); 7] = [
            ("integer, generic (k=0)", Kind::Integer, Some(0)),
            ("integer, near (k=64)", Kind::Integer, Some(64)),
            ("integer, degenerate", Kind::Integer, None),
            ("rational, near (k=64)", Kind::Rational, Some(64)),
            ("rational, degenerate", Kind::Rational, None),
            ("algebraic, near (k=64)", Kind::Algebraic, Some(64)),
            (
                "transcendental, near (k=64)",
                Kind::Transcendental,
                Some(64),
            ),
        ];
        for (label, kind, k) in series {
            det.sweep(
                atlas,
                label,
                &bits,
                kind,
                |_| k,
                |b, rng| Frame::widened(kind, base, b as u32, rng),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// d-dimensional predicates.

fn nd_build(v: Vec<Real>) -> Vec<Vec<Real>> {
    // (d + 1) points of dimension d: d^2 + d terms.
    let d = (1..).find(|d| d * d + d >= v.len()).unwrap();
    let mut it = v.into_iter();
    (0..=d).map(|_| it.by_ref().take(d).collect()).collect()
}

fn nd_sphere_build(v: Vec<Real>) -> (Vec<Vec<Real>>, Vec<Real>) {
    // (d + 1) simplex points and a query: d^2 + 2d terms.
    let d = (1..).find(|d| d * d + 2 * d >= v.len()).unwrap();
    let mut it = v.into_iter();
    let simplex = (0..=d).map(|_| it.by_ref().take(d).collect()).collect();
    (simplex, it.collect())
}

/// `d + 1` points in `R^d`: the last is an integer affine combination of the
/// others plus `delta` along the last axis (`None`: a random point).
fn nd_points(rng: &mut Rng, d: usize, delta: Option<i64>) -> Vec<Vec<Int>> {
    let bits = 44;
    let mut pts: Vec<Vec<Int>> = (0..d)
        .map(|_| (0..d).map(|_| signed(rng, bits)).collect())
        .collect();
    let last = match delta {
        None => (0..d).map(|_| signed(rng, bits)).collect(),
        Some(delta) => {
            let mut coefficients: Vec<i64> = (1..d).map(|_| rng.range(-1, 2)).collect();
            coefficients.insert(0, 1 - coefficients.iter().sum::<i64>());
            let mut last: Vec<Int> = (0..d)
                .map(|axis| (0..d).map(|i| &pts[i][axis] * coefficients[i]).sum())
                .collect();
            last[d - 1] += delta;
            last
        }
    };
    pts.push(last);
    pts
}

fn nd_terms(frame: &Frame, pts: &[Vec<Int>]) -> Vec<Term> {
    pts.iter()
        .flat_map(|p| {
            p.iter()
                .enumerate()
                .map(|(axis, n)| frame.coord(axis.min(2), n))
        })
        .collect()
}

fn nd_predicates(atlas: &mut Atlas) {
    let dims: Vec<f64> = (2..=9).map(f64::from).collect();
    let series: [(&str, Kind, Option<i64>); 7] = [
        ("f64, generic", Kind::F64, None),
        ("f64, near (δ=1, k≈44)", Kind::F64, Some(1)),
        ("f64, degenerate", Kind::F64, Some(0)),
        ("integer, near (δ=1)", Kind::Integer, Some(1)),
        ("rational, near (δ=1)", Kind::Rational, Some(1)),
        ("algebraic, near (δ=1)", Kind::Algebraic, Some(1)),
        ("transcendental, near (δ=1)", Kind::Transcendental, Some(1)),
    ];
    atlas.operation(
        "d-dimensional",
        "orient_d by dimension",
        "dimension d",
        "Sign of the (d+1)-point orientation determinant in R^d with 44-bit lattice coordinates: generic, one lattice step off a hyperplane, and exactly on it. The last series is affine_independent_d on the near inputs.",
    );
    for (label, kind, delta) in series {
        atlas.sweep(
            label,
            &dims,
            |d, rng| {
                let pts = nd_points(rng, d as usize, delta);
                Input::new(
                    kind,
                    nd_terms(&Frame::new(kind, 44, 0, rng), &pts),
                    nd_build,
                )
            },
            |input| input.run(|pts, pol| orient_d(pts, pol).value()),
        );
    }
    atlas.sweep(
        "f64 near, affine_independent_d",
        &dims,
        |d, rng| {
            let pts = nd_points(rng, d as usize, Some(1));
            Input::new(
                Kind::F64,
                nd_terms(&Frame::new(Kind::F64, 44, 0, rng), &pts),
                nd_build,
            )
        },
        |input| input.run(|pts, pol| affine_independent_d(pts, pol).value()),
    );

    atlas.operation(
        "d-dimensional",
        "insphere_d by dimension",
        "dimension d",
        "In-sphere test in R^d: a simplex on a sphere of radius ≈ 2^40 around a 44-bit center, and a query inside, one lattice step outside, or exactly on the sphere.",
    );
    for (label, kind, delta) in series {
        atlas.sweep(
            label,
            &dims,
            |d, rng| {
                let d = d as usize;
                let center: Vec<Int> = (0..d).map(|_| signed(rng, 44)).collect();
                let radius = odd(rng, 40);
                let at = |axis: usize, r: &Int| -> Vec<Int> {
                    let mut p = center.clone();
                    p[axis] += r;
                    p
                };
                let mut pts: Vec<Vec<Int>> = (0..d).map(|i| at(i, &radius)).collect();
                pts.push(at(0, &-radius.clone()));
                pts.push(match delta {
                    None => at(1, &-(&radius / Int::from(2))),
                    Some(delta) => at(1, &-(&radius + delta)),
                });
                Input::new(
                    kind,
                    nd_terms(&Frame::new(kind, 44, 0, rng), &pts),
                    nd_sphere_build,
                )
            },
            |input| input.run(|(simplex, q), pol| insphere_d(simplex, q, pol).value()),
        );
    }
}

// ---------------------------------------------------------------------------
// Composite predicates, swept over distance to degeneracy.

/// Sweep each of `kinds` over the degeneracy axis with a lattice construction
/// `make(rng, k) -> (points, planes, squared scalars, lengths)`.
fn derived<P: FromReals, const D: usize, R: perf_atlas::Outcome>(
    atlas: &mut Atlas,
    kinds: &[Kind],
    axis: &[f64],
    base: u32,
    make: fn(&mut Rng, Option<u32>) -> Shape<D>,
    build: fn(Vec<Real>) -> Geo<P>,
    run: fn(&Geo<P>, PredicatePolicy) -> R,
) {
    for &kind in kinds {
        atlas.sweep(
            kind.label(),
            axis,
            |m, rng| {
                let shape = make(rng, level(m));
                let frame = Frame::new(kind, base, 0, rng);
                let mut terms = frame.points(&shape.points);
                for (n, off) in &shape.planes {
                    terms.extend(frame.plane(n, off));
                }
                terms.extend(shape.squared.iter().map(|n| frame.squared(n)));
                terms.extend(shape.lengths.iter().map(|n| frame.length(n)));
                Input::new(kind, terms, build)
            },
            |input| input.run(run),
        );
    }
}

/// Lattice data of a composite input.
struct Shape<const D: usize> {
    points: Vec<[Int; D]>,
    planes: Vec<([Int; 3], Int)>,
    squared: Vec<Int>,
    lengths: Vec<Int>,
}

impl<const D: usize> Shape<D> {
    fn points(points: Vec<[Int; D]>) -> Self {
        Self {
            points,
            planes: Vec::new(),
            squared: Vec::new(),
            lengths: Vec::new(),
        }
    }
}

fn perp(u: &[Int; 2]) -> [Int; 2] {
    [-u[1].clone(), u[0].clone()]
}

fn composite_2d(atlas: &mut Atlas) {
    let axis = degeneracy_axis(96);
    atlas.operation(
        "2D composites",
        "classify_point_segment",
        DEGENERACY,
        "Point against a closed segment, the point projecting inside the segment at relative distance ≈2^-k from its line.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT2_BITS,
        |rng, k| Shape::points(near_collinear2(rng, ORIENT2_BITS, k).0.to_vec()),
        |v| geo::<Point2>(v, 3, 0),
        |g, pol| classify_point_segment(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    atlas.operation(
        "2D composites",
        "classify_segment_intersection",
        DEGENERACY,
        "Two segments where an endpoint of the second lies ≈2^-k (relative) from the first segment's interior; degenerate is a T-junction.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT2_BITS,
        |rng, k| {
            let ([a, b, c], u) = near_collinear2(rng, ORIENT2_BITS, k);
            let d = add(&c, &perp(&u));
            Shape::points(vec![a, b, c, d])
        },
        |v| geo::<Point2>(v, 4, 0),
        |g, pol| classify_segment_intersection(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
    );

    atlas.operation(
        "2D composites",
        "classify_point_triangle",
        DEGENERACY,
        "Point against a triangle, ≈2^-k (relative) from the interior of one edge; degenerate is on the edge.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT2_BITS,
        |rng, k| {
            let ([a, b, c], u) = near_collinear2(rng, ORIENT2_BITS, k);
            let e = add(&a, &perp(&u));
            Shape::points(vec![a, b, e, c])
        },
        |v| geo::<Point2>(v, 4, 0),
        |g, pol| classify_point_triangle(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
    );

    atlas.operation(
        "2D composites",
        "compare_point2_distance_squared",
        DEGENERACY,
        "Which of two points is closer to an anchor, with squared distances differing by ≈2^-k (relative).",
    );
    derived(
        atlas,
        &KINDS,
        &axis[..axis.len().min(24)]
            .iter()
            .copied()
            .chain([DEG])
            .collect::<Vec<_>>(),
        CIRCLE_BITS,
        |rng, k| {
            let c = near_cocircular2(rng, k);
            Shape::points(vec![c.center, c.points[0].clone(), c.points[3].clone()])
        },
        |v| geo::<Point2>(v, 3, 0),
        |g, pol| compare_point2_distance_squared(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    for (name, segment) in [
        ("classify_circle_line2", false),
        ("classify_circle_segment2", true),
    ] {
        atlas.operation(
            "2D composites",
            name,
            DEGENERACY,
            "Circle against a line (segment) at distance R from the center, with R^2 perturbed by ≈2^-k (relative); degenerate is tangency.",
        );
        let make: fn(&mut Rng, Option<u32>) -> Shape<2> = |rng, k| {
            let c = near_cocircular2(rng, Some(0));
            let w = sub(&c.points[0], &c.center);
            let a = add(&c.points[0], &perp(&w));
            let b = sub(&c.points[0], &perp(&w));
            let r2 = match k {
                None => c.radius_squared.clone(),
                Some(k) => &c.radius_squared + relative_delta(&c.radius_squared, k, rng),
            };
            Shape {
                squared: vec![r2],
                ..Shape::points(vec![c.center, a, b])
            }
        };
        if segment {
            derived(
                atlas,
                &KINDS,
                &axis,
                CIRCLE_BITS,
                make,
                |v| geo::<Point2>(v, 3, 0),
                |g, pol| classify_circle_segment2(&g.p[0], &g.s[0], &g.p[1], &g.p[2], pol).value(),
            );
        } else {
            derived(
                atlas,
                &KINDS,
                &axis,
                CIRCLE_BITS,
                make,
                |v| geo::<Point2>(v, 3, 0),
                |g, pol| classify_circle_line2(&g.p[0], &g.s[0], &g.p[1], &g.p[2], pol).value(),
            );
        }
    }
}

fn composite_3d(atlas: &mut Atlas) {
    let axis = degeneracy_axis(96);
    let sphere_axis = degeneracy_axis(92);
    atlas.operation(
        "3D composites",
        "classify_point_plane",
        DEGENERACY,
        "Point against an explicit plane n·x + d = 0 (24-bit normal, exact offset), at relative distance ≈2^-k (k ≤ 72).",
    );
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(72),
        48,
        |rng, k| {
            let (n, off, p, _) = near_plane(rng, 48, k);
            Shape {
                planes: vec![(n, off)],
                ..Shape::points(vec![p])
            }
        },
        |v| geo::<Point3>(v, 1, 1),
        |g, pol| classify_point_plane(&g.p[0], &g.planes[0], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_point_triangle3",
        DEGENERACY,
        "Point above the interior of a 3D triangle at relative distance ≈2^-k from its plane.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        |rng, k| {
            Shape::points(
                near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Inside)
                    .points
                    .to_vec(),
            )
        },
        |v| geo::<Point3>(v, 4, 0),
        |g, pol| classify_point_triangle3(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
    );

    let with_apex: fn(&mut Rng, Option<u32>) -> Shape<3> = |rng, k| {
        let t = near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Inside);
        let [a, b, c, d] = t.points;
        let e = add(&d, &t.apex_offset);
        let f = add(&e, &t.in_plane);
        Shape::points(vec![a, b, c, d, e, f])
    };

    atlas.operation(
        "3D composites",
        "classify_point_tetrahedron",
        DEGENERACY,
        "Point ≈2^-k (relative) from the interior of one face of a well-shaped tetrahedron.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        with_apex,
        |v| geo::<Point3>(v, 6, 0),
        |g, pol| {
            let p = &g.p;
            classify_point_tetrahedron(&p[0], &p[1], &p[2], &p[4], &p[3], pol).value()
        },
    );

    atlas.operation(
        "3D composites",
        "classify_segment_triangle3_intersection",
        DEGENERACY,
        "Segment from a point ≈2^-k (relative) off a triangle's interior to a far point; degenerate is an endpoint on the triangle.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        with_apex,
        |v| geo::<Point3>(v, 6, 0),
        |g, pol| {
            let p = &g.p;
            classify_segment_triangle3_intersection(&p[3], &p[4], &p[0], &p[1], &p[2], pol).value()
        },
    );

    atlas.operation(
        "3D composites",
        "classify_ray_triangle3_intersection",
        DEGENERACY,
        "Ray whose origin lies ≈2^-k (relative) off a triangle's interior, pointing away along the normal side.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        |rng, k| {
            let mut shape = with_apex_shape(rng, k);
            let direction = sub(&shape.points[4], &shape.points[3]);
            shape.lengths = direction.to_vec();
            shape
        },
        |v| geo::<Point3>(v, 6, 0),
        |g, pol| {
            let p = &g.p;
            let direction = Point3::new(g.s[0].clone(), g.s[1].clone(), g.s[2].clone());
            classify_ray_triangle3_intersection(&p[3], &direction, &p[0], &p[1], &p[2], pol).value()
        },
    );

    atlas.operation(
        "3D composites",
        "classify_triangle_triangle3",
        DEGENERACY,
        "Two triangles where a vertex of the second lies ≈2^-k (relative) off the first's interior and the rest is on one side.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        with_apex,
        |v| geo::<Point3>(v, 6, 0),
        |g, pol| {
            let p = &g.p;
            classify_triangle_triangle3(&p[0], &p[1], &p[2], &p[3], &p[4], &p[5], pol).value()
        },
    );

    atlas.operation(
        "3D composites",
        "classify_segment3_intersection",
        DEGENERACY,
        "Two 3D segments that cross in projection and are ≈2^-k (relative) from coplanar; degenerate is a proper crossing.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT3_BITS,
        |rng, k| {
            Shape::points(
                near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Crossing)
                    .points
                    .to_vec(),
            )
        },
        |v| geo::<Point3>(v, 4, 0),
        |g, pol| classify_segment3_intersection(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "compare_point3_distance_squared",
        DEGENERACY,
        "Which of two points is closer to an anchor, with squared distances differing by ≈2^-k (relative).",
    );
    derived(
        atlas,
        &KINDS,
        &sphere_axis,
        SPHERE_BITS,
        |rng, k| {
            let s = near_cospherical3(rng, k);
            Shape::points(vec![s.center, s.points[0].clone(), s.points[4].clone()])
        },
        |v| geo::<Point3>(v, 3, 0),
        |g, pol| compare_point3_distance_squared(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_point_sphere3",
        DEGENERACY,
        "Point on a sphere of squared radius R^2 classified against R^2 perturbed by ≈2^-k (relative).",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        SPHERE_BITS,
        |rng, k| {
            let s = near_cospherical3(rng, Some(0));
            let r2 = match k {
                None => s.radius_squared.clone(),
                Some(k) => &s.radius_squared + relative_delta(&s.radius_squared, k, rng),
            };
            Shape {
                squared: vec![r2],
                ..Shape::points(vec![s.center, s.points[0].clone()])
            }
        },
        |v| geo::<Point3>(v, 2, 0),
        |g, pol| classify_point_sphere3(&g.p[0], &g.s[0], &g.p[1], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_sphere3_intersection",
        DEGENERACY,
        "Two spheres whose radii sum to the center distance up to ≈2^-k (relative, k ≤ 44); degenerate is external tangency.",
    );
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(44),
        SPHERE_BITS,
        |rng, k| {
            let s = near_cospherical3(rng, Some(0));
            let r1 = &s.radius / 3;
            let mut r2 = &s.radius - &r1;
            if let Some(k) = k {
                r2 += relative_delta(&s.radius, k + 2, rng);
            }
            Shape {
                lengths: vec![r1, r2],
                ..Shape::points(vec![s.center, s.points[0].clone()])
            }
        },
        |v| geo::<Point3>(v, 2, 0),
        |g, pol| classify_sphere3_intersection(&g.p[0], &g.s[0], &g.p[1], &g.s[1], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "compare_point_segment3_distance_squared",
        DEGENERACY,
        "Squared distance from a point to a segment interior compared with a threshold that differs by ≈2^-k (relative).",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        42,
        |rng, k| {
            let a: [Int; 3] = rand_vec(rng, 42);
            let h: [Int; 3] = rand_vec(rng, 42);
            let r: [Int; 3] = rand_vec(rng, 8);
            let foot = add(&a, &h);
            let normal = cross(&h, &r);
            let p = add(&foot, &normal);
            let d2 = dot(&normal, &normal);
            let t2 = match k {
                None => d2.clone(),
                Some(k) => &d2 + relative_delta(&d2, k, rng),
            };
            Shape {
                squared: vec![t2],
                ..Shape::points(vec![p, a.clone(), add(&foot, &h)])
            }
        },
        |v| geo::<Point3>(v, 3, 0),
        |g, pol| {
            compare_point_segment3_distance_squared(&g.p[0], &g.p[1], &g.p[2], &g.s[0], pol).value()
        },
    );

    atlas.operation(
        "3D composites",
        "compare_point_plane_distance_squared",
        DEGENERACY,
        "Squared distance from a point to an explicit plane (integer-length normal) compared with a threshold differing by ≈2^-k (relative).",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        48,
        |rng, k| {
            // A normal of integer length: a rotated (1, 4, 8).
            let s = near_cospherical3_small(rng);
            let (normal, length) = s;
            let p: [Int; 3] = rand_vec(rng, 48);
            let distance = big(rng, 40);
            let offset = &length * &distance - dot(&normal, &p);
            let d2 = &distance * &distance;
            let t2 = match k {
                None => d2.clone(),
                Some(k) => &d2 + relative_delta(&d2, k, rng),
            };
            Shape {
                planes: vec![(normal, offset)],
                squared: vec![t2],
                ..Shape::points(vec![p])
            }
        },
        |v| geo::<Point3>(v, 1, 1),
        |g, pol| compare_point_plane_distance_squared(&g.p[0], &g.planes[0], &g.s[0], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_triangle3_degeneracy",
        DEGENERACY,
        "Degeneracy class of a 3D triangle whose vertices are ≈2^-k (relative) from collinear; degenerate is a collinear triangle.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT2_BITS,
        |rng, k| Shape::points(near_collinear3(rng, k).to_vec()),
        |v| geo::<Point3>(v, 3, 0),
        |g, pol| classify_triangle3_degeneracy(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_point_segment3",
        DEGENERACY,
        "Point against a closed 3D segment, projecting inside it at relative distance ≈2^-k from its line.",
    );
    derived(
        atlas,
        &KINDS,
        &axis,
        ORIENT2_BITS,
        |rng, k| Shape::points(near_collinear3(rng, k).to_vec()),
        |v| geo::<Point3>(v, 3, 0),
        |g, pol| classify_point_segment3(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "compare_point3_lexicographic",
        DEGENERACY,
        "Lexicographic order of two points equal in x and y whose z differ by ≈2^-k of the coordinate scale (k ≤ 44); degenerate points are equal.",
    );
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(44),
        48,
        |rng, k| {
            let a: [Int; 3] = rand_vec(rng, 48);
            let mut b = a.clone();
            if let Some(k) = k {
                b[2] += relative_delta(&pow2(48), k, rng);
            }
            Shape::points(vec![a, b])
        },
        |v| geo::<Point3>(v, 2, 0),
        |g, pol| compare_point3_lexicographic(&g.p[0], &g.p[1], pol).value(),
    );

    atlas.operation(
        "3D composites",
        "classify_homogeneous_point_plane",
        DEGENERACY,
        "Side of an explicit plane for the homogeneous intersection point of three other planes, the plane passing ≈2^-k (relative, k ≤ 72) from it. The point is constructed outside the timer for exact kinds.",
    );
    for kind in KINDS {
        atlas.sweep(
            kind.label(),
            &degeneracy_axis(72),
            |m, rng| {
                let frame = Frame::new(kind, 48, 0, rng);
                let point: [Int; 3] = rand_vec(rng, 48);
                let mut terms = Vec::new();
                for i in 0..4 {
                    let normal: [Int; 3] = rand_vec(rng, 24);
                    let t = match (i, level(m)) {
                        (3, Some(k)) => {
                            let bits = 72 - k.min(71);
                            if rng.coin() { pow2(bits) } else { -pow2(bits) }
                        }
                        _ => Int::zero(),
                    };
                    let offset = t - dot(&normal, &point);
                    terms.extend(frame.plane(&normal, &offset));
                }
                Input::new(kind, terms, |v| {
                    let g = geo::<Point3>(v, 0, 4);
                    let point = intersect_three_planes(&g.planes[0], &g.planes[1], &g.planes[2]);
                    (g, point)
                })
            },
            |input| {
                input.run(|(g, point), pol| {
                    classify_homogeneous_point_plane(point, &g.planes[3], pol).value()
                })
            },
        );
    }

    atlas.operation(
        "boxes",
        "classify_aabb3_intersection",
        DEGENERACY,
        "Two boxes separated along x by ≈2^-k of their extent (k ≤ 44); degenerate is face contact.",
    );
    let boxes: fn(&mut Rng, Option<u32>) -> Shape<3> = |rng, k| {
        let min: [Int; 3] = rand_vec(rng, 48);
        let extent: [Int; 3] = std::array::from_fn(|_| big(rng, 46));
        let max = add(&min, &extent);
        let gap = match k {
            None => Int::zero(),
            Some(k) => relative_delta(&extent[0], k, rng),
        };
        let min2 = [
            &max[0] + gap,
            &min[1] + (&extent[1] >> 2u32),
            min[2].clone(),
        ];
        let max2 = add(&min2, &extent);
        Shape::points(vec![min, max, min2, max2])
    };
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(44),
        48,
        boxes,
        |v| geo::<Point3>(v, 4, 0),
        |g, pol| classify_aabb3_intersection(&g.p[0], &g.p[1], &g.p[2], &g.p[3], pol).value(),
    );

    atlas.operation(
        "boxes",
        "classify_point_aabb3",
        DEGENERACY,
        "Point ≈2^-k of the box extent (k ≤ 44) from a box face; degenerate is on the face.",
    );
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(44),
        48,
        boxes,
        |v| geo::<Point3>(v, 4, 0),
        |g, pol| classify_point_aabb3(&g.p[0], &g.p[1], &g.p[2], pol).value(),
    );

    atlas.operation(
        "boxes",
        "classify_plane_aabb3",
        DEGENERACY,
        "Box whose nearest corner lies ≈2^-k (relative, k ≤ 72) from an explicit plane; degenerate is corner contact.",
    );
    derived(
        atlas,
        &KINDS,
        &degeneracy_axis(72),
        48,
        |rng, k| {
            let (n, off, corner, t) = near_plane(rng, 46, k);
            // Grow the box away from the plane on the corner's side.
            let side = if t.is_negative() {
                -Int::one()
            } else {
                Int::one()
            };
            let extent: [Int; 3] = std::array::from_fn(|i| sign_of(&n[i]) * &side * big(rng, 44));
            let other = add(&corner, &extent);
            let min: [Int; 3] = std::array::from_fn(|i| corner[i].clone().min(other[i].clone()));
            let max: [Int; 3] = std::array::from_fn(|i| corner[i].clone().max(other[i].clone()));
            Shape {
                planes: vec![(n, off)],
                ..Shape::points(vec![min, max])
            }
        },
        |v| geo::<Point3>(v, 2, 1),
        |g, pol| classify_plane_aabb3(&g.planes[0], &g.p[0], &g.p[1], pol).value(),
    );
}

/// A near-collinear 2D triple lifted to 3D by `z = alpha x + beta y` with
/// small integer slopes: every component of the cross product is a small
/// multiple of the 2D determinant.
fn near_collinear3(rng: &mut Rng, k: Option<u32>) -> [[Int; 3]; 3] {
    let (alpha, beta) = (rng.range(-3, 3), rng.range(-3, 3));
    let (points, _) = near_collinear2(rng, ORIENT2_BITS, k);
    points.map(|[x, y]| {
        let z = &x * alpha + &y * beta;
        [x, y, z]
    })
}

fn with_apex_shape(rng: &mut Rng, k: Option<u32>) -> Shape<3> {
    let t = near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Inside);
    let [a, b, c, d] = t.points;
    let e = add(&d, &t.apex_offset);
    let f = add(&e, &t.in_plane);
    Shape::points(vec![a, b, c, d, e, f])
}

/// A random integer vector of integer length: `(1, 4, 8)`-type points of
/// the radius-9 sphere rotated by a random 8-bit quaternion.
fn near_cospherical3_small(rng: &mut Rng) -> ([Int; 3], Int) {
    let [a, b, c, d]: [Int; 4] = rand_vec(rng, 8);
    let norm = &a * &a + &b * &b + &c * &c + &d * &d;
    let v = ints(sphere_9()[rng.below(sphere_9().len() as u64) as usize]);
    let rot = [
        [
            &a * &a + &b * &b - &c * &c - &d * &d,
            (&b * &c - &a * &d) * 2,
            (&b * &d + &a * &c) * 2,
        ],
        [
            (&b * &c + &a * &d) * 2,
            &a * &a - &b * &b + &c * &c - &d * &d,
            (&c * &d - &a * &b) * 2,
        ],
        [
            (&b * &d - &a * &c) * 2,
            (&c * &d + &a * &b) * 2,
            &a * &a - &b * &b - &c * &c + &d * &d,
        ],
    ];
    let n: [Int; 3] = std::array::from_fn(|i| dot(&rot[i], &v));
    (n, norm * 9)
}

// ---------------------------------------------------------------------------
// Scalar predicates.

#[derive(Clone, Copy, PartialEq)]
enum Scalar {
    F64,
    Rational,
    Sqrt,
    SqrtStrict,
    Ln,
}

impl Scalar {
    fn label(self) -> &'static str {
        match self {
            Scalar::F64 => "f64",
            Scalar::Rational => "rational",
            Scalar::Sqrt => "sqrt",
            Scalar::SqrtStrict => "sqrt strict",
            Scalar::Ln => "ln",
        }
    }
    fn policy(self) -> PredicatePolicy {
        if self == Scalar::SqrtStrict {
            STRICT
        } else {
            APPROX
        }
    }
}

const SCALARS: [Scalar; 5] = [
    Scalar::F64,
    Scalar::Rational,
    Scalar::Sqrt,
    Scalar::SqrtStrict,
    Scalar::Ln,
];

/// Two values `a < b` with `b / a - 1 ≈ 2^-k`, or equal values written as
/// different expressions when degenerate: `(sqrt(x) sqrt(y), sqrt(x y))` and
/// `(ln x + ln y, ln(x y))`. Roots and logarithms are built inside the timed
/// call.
struct Pair {
    kind: Scalar,
    x: Rational,
    y: Rational,
    a: f64,
    b: f64,
    degenerate: bool,
}

impl Pair {
    fn make(kind: Scalar, k: Option<u32>, rng: &mut Rng) -> Self {
        // x in [1, 2) with 40-bit parts.
        let x = &dyadic(&big(rng, 40), -40) + &Rational::one();
        let a = 1.0 + rng.unit();
        match k {
            Some(k) => {
                // y = x (1 + m 2^-(k+7)), m an 8-bit integer: y/x - 1 ≈ 2^-k.
                let bump = dyadic(&big(rng, 8), -(i64::from(k) + 7));
                Pair {
                    kind,
                    y: &x * &(&Rational::one() + &bump),
                    x,
                    a,
                    b: a * (1.0 + 2f64.powi(-(k as i32))),
                    degenerate: false,
                }
            }
            None => Pair {
                kind,
                y: if kind == Scalar::Rational {
                    x.clone()
                } else {
                    &dyadic(&big(rng, 30), -30) + &Rational::one()
                },
                x,
                a,
                b: a,
                degenerate: true,
            },
        }
    }

    fn values(&self) -> (Real, Real) {
        let x = || Real::new(self.x.clone());
        let y = || Real::new(self.y.clone());
        match self.kind {
            Scalar::F64 => (
                Real::try_from(self.a).unwrap(),
                Real::try_from(self.b).unwrap(),
            ),
            Scalar::Rational => (x(), y()),
            Scalar::Sqrt | Scalar::SqrtStrict if self.degenerate => (
                &x().sqrt().unwrap() * &y().sqrt().unwrap(),
                (&x() * &y()).sqrt().unwrap(),
            ),
            Scalar::Sqrt | Scalar::SqrtStrict => (x().sqrt().unwrap(), y().sqrt().unwrap()),
            Scalar::Ln if self.degenerate => (
                &x().ln().unwrap() + &y().ln().unwrap(),
                (&x() * &y()).ln().unwrap(),
            ),
            Scalar::Ln => (x().ln().unwrap(), y().ln().unwrap()),
        }
    }
}

fn scalar_predicates(atlas: &mut Atlas) {
    let axis = degeneracy_axis(96);
    let f64_axis = degeneracy_axis(52);
    type Run = fn(&Real, &Real, PredicatePolicy) -> bool;
    let ops: [(&str, &str, Run); 4] = [
        (
            "compare_reals",
            "Order of a and b = a(1 + 2^-k); degenerate pairs are equal values written differently (sqrt(x)sqrt(y) vs sqrt(xy), ln x + ln y vs ln xy). Roots and logarithms are built in the call.",
            |a, b, pol| compare_reals(a, b, pol).value().is_some(),
        ),
        (
            "classify_real_sign",
            "Sign of b - a for the same pairs, the difference built in the call.",
            |a, b, pol| classify_real_sign(&(b - a), pol).value().is_some(),
        ),
        (
            "classify_ball_sign",
            "Sign of the ball with center b and radius a, whose lower end b - a is ≈2^-k (relative) above zero; degenerate balls touch zero, so their sign is undecidable.",
            |a, b, pol| classify_ball_sign(b, a, pol).value().is_some(),
        ),
        (
            "classify_real_closed_interval",
            "Location of b relative to the closed interval [a/2, a] whose endpoint a is ≈2^-k (relative) away.",
            |a, b, pol| {
                let half = a * &Real::new(Rational::fraction(1, 2).unwrap());
                classify_real_closed_interval(b, &half, a, pol)
                    .value()
                    .is_some()
            },
        ),
    ];
    for (name, description, run) in ops {
        atlas.operation("scalars", name, DEGENERACY, description);
        for kind in SCALARS {
            let axis = if kind == Scalar::F64 {
                &f64_axis
            } else {
                &axis
            };
            atlas.sweep(
                kind.label(),
                axis,
                |m, rng| Pair::make(kind, level(m), rng),
                |pair| {
                    let (a, b) = pair.values();
                    run(&a, &b, pair.kind.policy())
                },
            );
        }
    }

    atlas.operation(
        "scalars",
        "certified_interval_sign",
        DEGENERACY,
        "Certified sign of the interval [b - a, b], whose lower end is ≈2^-k (relative) above zero, from endpoint facts; None (flagged) means the filter did not certify a sign, as for the degenerate interval that touches zero.",
    );
    for kind in SCALARS {
        let axis = if kind == Scalar::F64 {
            &f64_axis
        } else {
            &axis
        };
        atlas.sweep(
            kind.label(),
            axis,
            |m, rng| Pair::make(kind, level(m), rng),
            |pair| {
                let (a, b) = pair.values();
                certified_interval_sign(&(&b - &a), &b, pair.kind.policy()).and_then(|o| o.value())
            },
        );
    }

    atlas.operation(
        "scalars",
        "reciprocal_real",
        "operand bits",
        "Exact reciprocal after certifying the operand nonzero, for operands with parts of b bits.",
    );
    let bits = powers_of_two(3, 13);
    for kind in [Scalar::Rational, Scalar::Sqrt, Scalar::Ln] {
        atlas.sweep(
            kind.label(),
            &bits,
            |b, rng| {
                let b = b as u32;
                ratio(&big(rng, b), &odd(rng, b))
            },
            |x| {
                let x = Real::new(x.clone());
                let x = match kind {
                    Scalar::Sqrt => x.sqrt().unwrap(),
                    Scalar::Ln => (&x + &Real::one()).ln().unwrap(),
                    _ => x,
                };
                reciprocal_real(&x, APPROX).ok().and_then(|o| o.value())
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Polygons and point sets.

/// A strictly convex counterclockwise polygon with `n` vertices on a circle
/// of radius ≈2^40 around a 44-bit center (rounded to the lattice).
fn convex_polygon(rng: &mut Rng, n: usize) -> (Vec<[Int; 2]>, [Int; 2]) {
    let center: [Int; 2] = rand_vec(rng, 44);
    let radius = 2f64.powi(40) * (1.0 + rng.unit());
    let phase = rng.unit() * std::f64::consts::TAU;
    let points = (0..n)
        .map(|i| {
            let angle = phase + std::f64::consts::TAU * (i as f64 + 0.3 * rng.unit()) / n as f64;
            [
                &center[0] + Int::from((radius * angle.cos()).round() as i64),
                &center[1] + Int::from((radius * angle.sin()).round() as i64),
            ]
        })
        .collect();
    (points, center)
}

fn polygon_input(kind: Kind, n: usize, rng: &mut Rng) -> In2 {
    let (mut points, center) = convex_polygon(rng, n);
    points.push(center);
    let frame = Frame::new(kind, 46, 0, rng);
    Input::new(kind, frame.points(&points), |v| {
        let n = v.len() / 2;
        geo::<Point2>(v, n, 0)
    })
}

/// `n` planes tangent-ish to a sphere of radius ≈2^40 (16-bit normals) and
/// the sphere's center, which is strictly inside all of them.
fn convex_planes(kind: Kind, n: usize, rng: &mut Rng) -> In3 {
    let center: [Int; 3] = rand_vec(rng, 44);
    let frame = Frame::new(kind, 46, 0, rng);
    let mut terms = frame.points(std::slice::from_ref(&center));
    for _ in 0..n {
        let normal: [Int; 3] = rand_vec(rng, 16);
        let length = dot(&normal, &normal).sqrt() + 1;
        let offset = -dot(&normal, &center) - length * pow2(40);
        terms.extend(frame.plane(&normal, &offset));
    }
    Input::new(kind, terms, |v| {
        let n = (v.len() - 3) / 4;
        geo::<Point3>(v, 1, n)
    })
}

fn polygons(atlas: &mut Atlas) {
    let sizes = powers_of_two(2, 14);
    let kinds = [Kind::F64, Kind::Rational, Kind::Transcendental];
    type RingRun = fn(&[Point2], &Point2, PredicatePolicy) -> bool;
    let ops: [(&str, &str, RingRun); 4] = [
        (
            "ring_area_sign",
            "Sign of the area of a strictly convex n-gon with 46-bit lattice vertices.",
            |ring, _, pol| ring_area_sign(ring, pol).value().is_some(),
        ),
        (
            "ring_convexity",
            "Local convexity of the same n-gon (one orientation per vertex).",
            |ring, _, pol| ring_convexity(ring, pol) == hyperlimit::RingConvexity::LocallyConvex,
        ),
        (
            "classify_point_ring_even_odd",
            "Even-odd location of the polygon's center against the n-gon.",
            |ring, p, pol| classify_point_ring_even_odd(ring, p, pol).value().is_some(),
        ),
        (
            "classify_point_convex_polygon2",
            "Location of the center against the n-gon as a convex polygon.",
            |ring, p, pol| {
                classify_point_convex_polygon2(ring, p, pol)
                    .value()
                    .is_some()
            },
        ),
    ];
    for (name, description, run) in ops {
        atlas.operation("polygons and sets", name, "vertex count n", description);
        for kind in kinds {
            atlas.sweep(
                kind.label(),
                &sizes,
                |n, rng| polygon_input(kind, n as usize, rng),
                |input| {
                    input.run(|g, pol| {
                        let (ring, center) = g.p.split_at(g.p.len() - 1);
                        run(ring, &center[0], pol)
                    })
                },
            );
        }
    }

    atlas.operation(
        "polygons and sets",
        "classify_point_convex_planes3",
        "plane count n",
        "Location of a point strictly inside n halfspaces with 16-bit normals and exact offsets.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &powers_of_two(2, 12),
            |n, rng| convex_planes(kind, n as usize, rng),
            |input| {
                input.run(|g, pol| {
                    classify_point_convex_planes3(&g.planes, &g.p[0], pol)
                        .value()
                        .is_some()
                })
            },
        );
    }

    atlas.operation(
        "polygons and sets",
        "classify_halfspace_feasibility3",
        "plane count n",
        "Feasibility of n bounded 3D halfspaces (feasible, with an exact witness); the active-set search is polynomial in n.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &powers_of_two(2, 8),
            |n, rng| convex_planes(kind, n as usize, rng),
            |input| input.run(|g, pol| classify_halfspace_feasibility3(&g.planes, pol).value()),
        );
    }

    atlas.operation(
        "polygons and sets",
        "support_dop3_from_points",
        "point count n",
        "Exact 13-axis support k-DOP of n random 46-bit lattice points.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &powers_of_two(2, 13),
            |n, rng| {
                let points: Vec<[Int; 3]> = (0..n as usize).map(|_| rand_vec(rng, 46)).collect();
                let frame = Frame::new(kind, 46, 0, rng);
                Input::new(kind, frame.points(&points), |v| {
                    let n = v.len() / 3;
                    geo::<Point3>(v, n, 0)
                })
            },
            |input| {
                let axes: Vec<Point3> = [
                    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                    [1, 1, 1],
                    [1, -1, 1],
                    [1, 1, -1],
                    [1, -1, -1],
                    [1, 1, 0],
                    [1, -1, 0],
                    [1, 0, 1],
                    [1, 0, -1],
                    [0, 1, 1],
                    [0, 1, -1],
                ]
                .iter()
                .map(|a: &[i32; 3]| {
                    Point3::new(Real::from(a[0]), Real::from(a[1]), Real::from(a[2]))
                })
                .collect();
                input.run(|g, pol| support_dop3_from_points(&axes, &g.p, pol).value())
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Exact constructions.

fn constructions(atlas: &mut Atlas) {
    let bits: Vec<f64> = [64, 128, 256, 512, 1024, 2048, 4096]
        .map(f64::from)
        .to_vec();
    let kinds = [
        Kind::Integer,
        Kind::Rational,
        Kind::Algebraic,
        Kind::Transcendental,
    ];

    atlas.operation(
        "constructions",
        "construct_line_intersection_point",
        "coordinate bits",
        "Exact intersection point of two generic 2D lines through points with about b-bit coordinates.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &bits,
            |b, rng| {
                let points: Vec<[Int; 2]> = (0..4).map(|_| rand_vec(rng, 48)).collect();
                let frame = Frame::widened(kind, 48, b as u32, rng);
                Input::new(kind, frame.points(&points), |v| geo::<Point2>(v, 4, 0))
            },
            |input| {
                input.run(|g, pol| {
                    let p = &g.p;
                    construct_line_intersection_point_with_policy(&p[0], &p[1], &p[2], &p[3], pol)
                })
            },
        );
    }

    atlas.operation(
        "constructions",
        "intersect_three_planes",
        "coordinate bits",
        "Homogeneous intersection point of three explicit planes (24-bit normals) through points with about b-bit coordinates.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &bits,
            |b, rng| {
                let frame = Frame::widened(kind, 48, b as u32, rng);
                let mut terms = Vec::new();
                for _ in 0..3 {
                    let (n, off, _, _) = near_plane(rng, 48, None);
                    terms.extend(frame.plane(&n, &off));
                }
                Input::new(kind, terms, |v| geo::<Point3>(v, 0, 3))
            },
            |input| {
                input.run(|g, _| {
                    Some(intersect_three_planes(
                        &g.planes[0],
                        &g.planes[1],
                        &g.planes[2],
                    ))
                })
            },
        );
    }

    atlas.operation(
        "constructions",
        "intersect_segment_with_plane",
        "coordinate bits",
        "Exact crossing point of a segment through an explicit plane (24-bit normal), coordinates of about b bits.",
    );
    for kind in kinds {
        atlas.sweep(
            kind.label(),
            &bits,
            |b, rng| {
                let frame = Frame::widened(kind, 48, b as u32, rng);
                let (n, off, p0, t) = near_plane(rng, 48, Some(24));
                let n2 = dot(&n, &n);
                let lambda: Int = -(sign_of(&t) * (t.abs() / &n2 + Int::from(2)));
                let p1 = add(&p0, &scale(&n, &lambda));
                let mut terms = frame.points(&[p0, p1]);
                terms.extend(frame.plane(&n, &off));
                Input::new(kind, terms, |v| geo::<Point3>(v, 2, 1))
            },
            |input| {
                input.run(|g, pol| {
                    intersect_segment_with_plane(&g.planes[0], &g.p[0], &g.p[1], pol).relation
                        == SegmentPlaneRelation::ProperCrossing
                })
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Batches.

fn batches(atlas: &mut Atlas) {
    let sizes = powers_of_two(0, 14);
    atlas.operation(
        "batches",
        "orient2 batch",
        "batch size",
        "orient2_batch over cases with mixed distance to degeneracy (k uniform in 0..96), sequential and rayon-parallel; time is per batch.",
    );
    for kind in [Kind::F64, Kind::Rational] {
        for parallel in BATCH_MODES {
            let label = format!(
                "{} {}",
                kind.label(),
                if parallel { "parallel" } else { "sequential" }
            );
            atlas.sweep(
                &label,
                &sizes,
                |n, rng| {
                    (0..n as usize)
                        .map(|_| {
                            let k = rng.below(97) as u32;
                            let pts = near_collinear2(rng, ORIENT2_BITS, Some(k)).0;
                            let frame = Frame::new(kind, ORIENT2_BITS, 0, rng);
                            let mut g =
                                Input::new(kind, frame.points(&pts), |v| geo::<Point2>(v, 3, 0))
                                    .warm
                                    .expect("warm kinds")
                                    .p
                                    .into_iter();
                            (g.next().unwrap(), g.next().unwrap(), g.next().unwrap())
                        })
                        .collect::<Vec<_>>()
                },
                |cases| {
                    let out = match parallel {
                        #[cfg(feature = "parallel")]
                        true => orient2_batch_parallel(cases, APPROX),
                        _ => orient2_batch(cases, APPROX),
                    };
                    out.iter().all(|o| o.value().is_some())
                },
            );
        }
    }

    atlas.operation(
        "batches",
        "insphere3 batch",
        "batch size",
        "insphere3_batch over cases with mixed distance to degeneracy (k uniform in 0..92), sequential and rayon-parallel; time is per batch.",
    );
    for kind in [Kind::F64, Kind::Rational] {
        for parallel in BATCH_MODES {
            let label = format!(
                "{} {}",
                kind.label(),
                if parallel { "parallel" } else { "sequential" }
            );
            atlas.sweep(
                &label,
                &powers_of_two(0, 12),
                |n, rng| {
                    (0..n as usize)
                        .map(|_| {
                            let k = rng.below(93) as u32;
                            let pts = near_cospherical3(rng, Some(k)).points;
                            let frame = Frame::new(kind, SPHERE_BITS, 0, rng);
                            let mut g =
                                Input::new(kind, frame.points(&pts), |v| geo::<Point3>(v, 5, 0))
                                    .warm
                                    .expect("warm kinds")
                                    .p
                                    .into_iter();
                            (
                                g.next().unwrap(),
                                g.next().unwrap(),
                                g.next().unwrap(),
                                g.next().unwrap(),
                                g.next().unwrap(),
                            )
                        })
                        .collect::<Vec<_>>()
                },
                |cases| {
                    let out = match parallel {
                        #[cfg(feature = "parallel")]
                        true => insphere3_batch_parallel(cases, APPROX),
                        _ => insphere3_batch(cases, APPROX),
                    };
                    out.iter().all(|o| o.value().is_some())
                },
            );
        }
    }
}

/// Batch modes to sweep: the rayon-parallel batches exist only with the
/// `parallel` feature, which `scripts/perf-atlas/run.sh` enables.
#[cfg(feature = "parallel")]
const BATCH_MODES: [bool; 2] = [false, true];
#[cfg(not(feature = "parallel"))]
const BATCH_MODES: [bool; 1] = [false];

fn main() {
    let mut atlas = Atlas::from_args("hyperlimit");
    determinant_predicates(&mut atlas);
    nd_predicates(&mut atlas);
    composite_2d(&mut atlas);
    composite_3d(&mut atlas);
    scalar_predicates(&mut atlas);
    polygons(&mut atlas);
    constructions(&mut atlas);
    batches(&mut atlas);
    atlas.finish();
}
