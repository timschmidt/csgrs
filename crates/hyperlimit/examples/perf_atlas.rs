//! Performance atlas for selected Hyperlimit predicate fixtures and scalar recipes.
//!
//! Sweeps nominal degeneracy, coordinate magnitude/bit length, and input size.
//! Lattice generators retain exact determinant or radial relations where described;
//! their labels are not universal guarantees about Euclidean distance, conditioning,
//! all scalar representations, or completed geometric contracts. Some randomized
//! searches have no iteration/work bound and use probable-prime screening.
//!
//! Frame converts integer lattices into binary64-backed, integer, fractional, or
//! symbolic scale/translation recipes; separate trigonometric recipes use rational
//! angles in radians. Input rebuilds non-ready terms with fresh local atom cells on
//! each call, while ready geometry is retained within its Input. The default atlas
//! harness regenerates those Input objects outside timing for each call; symbolic
//! reconstruction remains inside timing. Neither strategy proves all kernel/shared
//! caches are cold. Selecting a policy and obtaining a value do not establish
//! a certified receipt or an
//! independently correct geometric answer.
//!
//! Atlas success flags depend on each callback's return contract. For example,
//! Option records presence and bool records truth, while numeric reference returns
//! use the atlas's always-success implementation. Failure marks are not a uniform
//! typed account of kernel refusal, and timings alone do not validate results.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly with
//! `cargo run --release -p hyperlimit --all-features --example perf_atlas -- --out atlas.json`.
//! External containment is needed for fixture generation and individual calls;
//! no total generation/work/memory bound is enforced by the helpers below.

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

/// Arbitrary-size signed integer used for exact lattice construction and determinant relations.
type Int = BigInt;

/// Policy permitting the terminal approximation in ordinary atlas recipes.
const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
/// Policy requiring certified decisions in explicitly strict recipes.
const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

/// Axis sentinel interpreted as an exactly degenerate fixture rather than finite exponent 112.
const DEG: f64 = 112.0;
/// Maximum finite degeneracy exponent used by the standard axis helper.
const K_MAX: u32 = 96;

/// Display label for the nominal relative-degeneracy axis and its sentinel convention.
const DEGENERACY: &str = "distance to degeneracy k (≈2^-k relative; 112 = exactly degenerate)";

/// Return levels 0,4,... up to the largest multiple of four ≤min(kmax,96), then 112.
///
/// The final value is a sentinel for exact degeneracy, not another finite exponent.
fn degeneracy_axis(kmax: u32) -> Vec<f64> {
    (0..=kmax.min(K_MAX))
        .step_by(4)
        .map(f64::from)
        .chain([DEG])
        .collect()
}

/// Decode values below 112 as a u32 exponent; all others, including NaN, become None.
///
/// The cast truncates finite fractions and saturates negative values to zero.
/// Intended callers pass the nonnegative axis levels; this is not input validation.
fn level(k: f64) -> Option<u32> {
    (k < DEG).then_some(k as u32)
}

// ---------------------------------------------------------------------------
// Big-integer helpers.

/// Draw a positive integer with exactly max(bits,1) bits from the shared RNG's limb recipe.
///
/// The top bit is set, so even bits=0 yields one. Allocation/work grows with bits;
/// there is no independent helper budget or cryptographic randomness guarantee.
fn big(rng: &mut Rng, bits: u32) -> Int {
    Int::from(BigUint::from_slice(&rng.limbs(bits)))
}

/// Draw a nonzero integer with max(bits,1) magnitude bits, then select its sign by an RNG bit.
fn signed(rng: &mut Rng, bits: u32) -> Int {
    let n = big(rng, bits);
    if rng.coin() { -n } else { n }
}

/// Draw a positive max(bits,1)-bit integer and force its low bit to one.
///
/// Used for nonzero odd denominators/factors; this is not a primality test.
fn odd(rng: &mut Rng, bits: u32) -> Int {
    big(rng, bits) | Int::one()
}

/// Allocate the exact integer 2^e by shifting one, without an exponent/resource cap.
fn pow2(e: u32) -> Int {
    Int::one() << e
}

/// Return -1 for a negative integer and +1 for zero or a positive integer.
///
/// The zero convention is intentional for choosing search directions/apex components;
/// this is not a three-valued mathematical sign function.
fn sign_of(n: &Int) -> Int {
    if n.is_negative() {
        -Int::one()
    } else {
        Int::one()
    }
}

/// Return a process-lifetime cached list of all primes from 2 through 1999.
///
/// The first call trial-divides each candidate through its square root; OnceLock
/// shares the completed vector across threads. No reset or eviction is provided.
fn small_primes() -> &'static [u32] {
    /// Process-lifetime initialized prime table, shared safely across callers.
    static PRIMES: OnceLock<Vec<u32>> = OnceLock::new();
    PRIMES.get_or_init(|| {
        (2u32..2000)
            .filter(|&n| (2..n).take_while(|d| d * d <= n).all(|d| n % d != 0))
            .collect()
    })
}

/// Screen for probable primality using small-prime division and twelve fixed Miller–Rabin bases.
///
/// Rejects integers below two, accepts cached small primes, then checks bases
/// 2,3,5,7,11,13,17,19,23,29,31,37. Execution is deterministic, but this code
/// supplies no arbitrary-size primality proof or certified bound for those bases.
/// A true result is candidate evidence for fixture search, not a general primality
/// certificate. Modular exponentiation has no independent work limit.
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

/// Find x,y with x²+y²=p using a square root of -1 and Euclidean reduction.
///
/// The intended input is a positive prime p≡1 mod 4. Searches bases without a
/// limit, selects the smaller modular root, then performs a Cornacchia-style
/// reduction and integer square root. It asserts the final sum-of-squares identity.
/// Preconditions are not validated here; invalid/composite candidates can hang or
/// panic. Callers use probable-prime screening, so that assertion remains necessary.
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

/// Search for x²+y²+z²=n by reducing a random square to a prime-candidate remainder.
///
/// For positive n≡1 or 2 mod 4, samples x below floor(sqrt(n)), adjusts its parity,
/// and seeks a positive remainder congruent to one mod four that passes is_prime.
/// The two_squares assertion checks the accepted remainder. No attempt/time budget
/// or universal termination guarantee is enforced: even small admissible residues
/// such as n=1 or n=2 cannot succeed through this search. Zero/negative or other
/// invalid inputs can panic or loop; production fixtures use large positive n.
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

/// Permute integer coordinates using descending swaps, then independently choose their signs.
///
/// Preserves squared norm exactly. RNG bounded draws use remainder reduction, so
/// no uniform distribution claim is made; zero-length arrays are returned unchanged.
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

/// Add equal-dimensional integer vectors componentwise, preserving exact values.
fn add<const D: usize>(a: &[Int; D], b: &[Int; D]) -> [Int; D] {
    std::array::from_fn(|i| &a[i] + &b[i])
}

/// Subtract equal-dimensional integer vectors componentwise, preserving exact values.
fn sub<const D: usize>(a: &[Int; D], b: &[Int; D]) -> [Int; D] {
    std::array::from_fn(|i| &a[i] - &b[i])
}

/// Multiply every vector component by the supplied integer, including zero or negative factors.
fn scale<const D: usize>(a: &[Int; D], s: &Int) -> [Int; D] {
    std::array::from_fn(|i| &a[i] * s)
}

/// Sum exact component products; the zero-dimensional dot product is zero.
fn dot<const D: usize>(a: &[Int; D], b: &[Int; D]) -> Int {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Return the exact right-handed cross product of two spatial integer vectors.
fn cross(a: &[Int; 3], b: &[Int; 3]) -> [Int; 3] {
    [
        &a[1] * &b[2] - &a[2] * &b[1],
        &a[2] * &b[0] - &a[0] * &b[2],
        &a[0] * &b[1] - &a[1] * &b[0],
    ]
}

/// Draw D signed nonzero integer components with max(bits,1) magnitude bits each.
fn rand_vec<const D: usize>(rng: &mut Rng, bits: u32) -> [Int; D] {
    std::array::from_fn(|_| signed(rng, bits))
}

/// Choose a nonzero signed integer perturbation relative to the bit length of m.
///
/// For k+1 below bit_length(m), draws exactly bit_length(m)-1-k magnitude bits;
/// otherwise returns ±1. This is a nominal scale near |m|·2^-k, with an integer
/// floor, not an exact relative ratio. Callers use positive bounded m and small k;
/// bit-count narrowing and k+1 are not guarded against extreme-range overflow.
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

/// Return the cached integer solutions of x²+y²=65² in lexicographic loop order.
///
/// Exhaustively enumerates coordinates -65..=65 once, retaining the vector for the
/// process lifetime. Consumers sample distinct indices without modifying this table.
fn circle_65() -> &'static [[i64; 2]] {
    /// Process-lifetime radius-65 lattice circle table, initialized on first use.
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

/// Return the cached integer solutions of x²+y²+z²=81 in nested-loop order.
///
/// Enumerates coordinates -9..=9 once and retains the vector for the process lifetime.
fn sphere_9() -> &'static [[i64; 3]] {
    /// Process-lifetime radius-nine lattice sphere table, initialized on first use.
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

/// Convert every i64 array component into the unbounded integer carrier exactly.
fn ints<const D: usize>(v: [i64; D]) -> [Int; D] {
    v.map(Int::from)
}

/// Sample count distinct indices by rejection from RNG draws below n.
///
/// Requires positive n and count≤n for the intended index contract; no explicit
/// validation or attempt bound exists. Oversized requests can loop indefinitely;
/// n=0 uses the RNG's zero fallback and can return an invalid zero index. Count
/// zero returns empty without a draw. Repeated linear membership checks add work.
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

/// Build an exact planar determinant fixture and return its displacement b-a.
///
/// For Some(k), requires k≤2l and chooses determinant ±2^(2l-k). A coprime pair
/// u=(ux,uy) permits solving ux·vy-uy·vx=s by modular inverse; the identity is
/// checked only by debug_assert. vx lies in [0,ux), which does not guarantee that
/// c's Euclidean orthogonal projection lies within segment ab.
///
/// For None, takes integer multiples m<n of one nonzero direction, so c is exactly
/// between a and b; this branch requires l≥12. Coprime-pair search has no iteration
/// bound. Small checked arithmetic ranges are a caller premise, not a resource cap.
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

/// Select a tetrahedron base combination before the controlled out-of-plane lift.
#[derive(Clone, Copy)]
enum Coplanar {
    /// Use arbitrary small integer in-plane coefficients before the vertical lift.
    Generic,
    /// Use quarter/quarter base coefficients in a four-times-expanded triangle before lifting.
    Inside,
    /// Arrange a segment crossing the first edge in the zero-lift case; nonzero lift need not intersect it.
    Crossing,
}

/// Exact tetrahedron sites plus displacements retained for constructing related fixtures.
struct Tetra {
    /// Three plane sites followed by the displaced or coplanar fourth point.
    points: [[Int; 3]; 4],
    /// Displacement with positive pairing against the oriented source-plane normal.
    apex_offset: [Int; 3],
    /// A transformed in-plane direction retained for additional fixture assembly.
    in_plane: [Int; 3],
}

/// Build a tetrahedron with controlled determinant magnitude, then apply a determinant-one shear.
///
/// For Some(k), requires k≤3l and constructs |det(u,v,w)|=2^(3l-k), multiplied
/// by 16 for Inside/Crossing. None makes the determinant zero. Shape selects an
/// in-plane base combination before a vertical lift; that lift is not generally a
/// normal displacement, so Euclidean projection/distance is not certified by the
/// shape name. Coprime search has no attempt bound.
///
/// A unit-lower times unit-upper integer map preserves determinant and incidence,
/// not angles or distances. Translation places the result at a random lattice
/// point. The returned apex displacement has positive normal pairing, and in_plane
/// retains a supporting direction for consumer fixture assembly.
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

/// Multiply random unit-lower and unit-upper 3×3 integer triangular matrices.
///
/// Each factor's off-diagonal entries are -1,0,1; product entries need not lie in
/// that range. Determinant is exactly +1, preserving lattice volume/orientation,
/// not Euclidean distance or all coordinate correlations.
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

/// Exact lattice circle support, its three sites, and the selected query.
struct Circle {
    /// Exact integer center shared by sites and radial construction.
    center: [Int; 2],
    /// Three distinct on-circle sites, then the query.
    points: [[Int; 2]; 4],
    /// Exact squared radius before the query perturbation.
    radius_squared: Int,
}

/// Build three exact circle sites and a query with an explicit squared-radius perturbation.
///
/// Rotates/scales radius-65 table points by a nonzero complex integer and translates
/// by a 51-bit center. Distinct site indices give three distinct circle points.
/// None selects a fourth on-circle point. Some(k) starts with relative_delta, then
/// steps outward in its sign until the new squared radius passes probable-prime
/// screening and a checked two-squares decomposition. Search changes the initial
/// perturbation; no exact proximity bound, iteration bound, or primality certificate
/// is returned. Successful decomposition preserves the claimed radial identity.
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

/// Exact lattice sphere support, four noncoplanar sites, and the selected query.
struct Sphere {
    /// Exact integer center shared by all source sites.
    center: [Int; 3],
    /// Four noncoplanar on-sphere sites, then the query.
    points: [[Int; 3]; 5],
    /// Square of radius, before any query perturbation.
    radius_squared: Int,
    /// Positive integer radius: nine times squared quaternion norm.
    radius: Int,
}

/// Build four noncoplanar exact sphere sites and a query at a selected squared radius.
///
/// An integer quaternion matrix scales radius-nine table points by its positive
/// squared quaternion norm, then translates by a 50-bit center. Rejection sampling
/// requires the first four table sites to have nonzero tetrahedron determinant.
/// None chooses another site on that sphere. Some(k) adjusts a signed relative
/// perturbation to residue one/two mod four, then searches a three-squares query.
/// The exact radius is nine times quaternion norm; returned squared radius is its
/// square. Searches have no attempt bound and proximity is nominal, not certified.
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

/// Return (normal,offset,point,t) with exact plane evaluation normal·point+offset=t.
///
/// Normal components have 24 magnitude bits and point components have max(l,1).
/// None gives t=0; Some(k) gives signed 2^max(1,l+24-k), so the finite sweep
/// saturates at magnitude two. This is a plane-equation value, not unit-normal signed
/// distance. Callers keep l+24 and shifts within their arithmetic/allocation range.
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

/// Select a finite atlas representation recipe and its predicate policy, not all scalar classes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    /// Framed dyadic coordinates imported from binary64 after explicit width checks.
    F64,
    /// Widened lattice integers without the frame dyadic scale or extra translation.
    Integer,
    /// Positive rational scale and per-axis rational translation with dyadic scaling.
    Rational,
    /// Scale by sqrt(2) and translate by sqrt(3) multiples.
    Algebraic,
    /// Scale by pi and translate by e multiples.
    Transcendental,
    /// Direct rational-angle trigonometric circle/sphere recipes under APPROXIMATE_512.
    Trig,
    /// The direct trigonometric recipe with STRICT decisions.
    TrigStrict,
}

impl Kind {
    /// Return the stable report label describing this recipe; labels do not certify coverage.
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

    /// Return STRICT only for TrigStrict; all other Kind variants use APPROXIMATE_512.
    fn policy(self) -> PredicatePolicy {
        if self == Kind::TrigStrict {
            STRICT
        } else {
            APPROX
        }
    }
}

/// Five framed kinds used for determinant sweeps, including raw lattice integers.
const DET_KINDS: [Kind; 5] = [
    Kind::F64,
    Kind::Integer,
    Kind::Rational,
    Kind::Algebraic,
    Kind::Transcendental,
];
/// Four framed kinds used by many composed sweeps; integer and trig recipes are omitted.
const KINDS: [Kind; 4] = [
    Kind::F64,
    Kind::Rational,
    Kind::Algebraic,
    Kind::Transcendental,
];

/// Named irrational constants reused within one Atoms instance.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Atom {
    /// Positive square root of two, used as a scale.
    Sqrt2,
    /// Positive square root of three, used as a translation atom.
    Sqrt3,
    /// Pi, used as a scale or to construct its square.
    Pi,
    /// Euler's number, used as a translation atom.
    E,
    /// Pi times itself for squared-length scaling.
    PiSquared,
}

/// Lazy per-instance scalar constants shared by terms in one reconstruction.
///
/// Input::run creates fresh cells for a rebuilt geometry. This does not clear
/// underlying kernel/global state or caches shared by cloned Ready terms.
#[derive(Default)]
struct Atoms {
    /// Lazy sqrt(2) value for this reconstruction.
    sqrt2: OnceCell<Real>,
    /// Lazy sqrt(3) value for this reconstruction.
    sqrt3: OnceCell<Real>,
    /// Lazy pi value shared with pi_squared construction.
    pi: OnceCell<Real>,
    /// Lazy e value for this reconstruction.
    e: OnceCell<Real>,
    /// Lazy product of the same local pi value with itself.
    pi_squared: OnceCell<Real>,
}

impl Atoms {
    /// Lazily construct and borrow the requested constant, reusing it in this cell set.
    ///
    /// PiSquared depends on the same Pi cell. Positive fixed square-root constructors
    /// are expected to succeed or panic. Cached values live as long as this Atoms.
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

/// One scalar recipe: retained ready value, linear combination of atoms, or rational-angle trig product.
#[derive(Clone)]
enum Term {
    /// Retain an existing scalar and clone it when evaluated.
    Ready(
        /// Scalar retained by the recipe, including any shared state it owns.
        Real,
    ),
    /// Sum rational multiples of named atoms in stored order.
    Sum(
        /// Ordered coefficient/atom pairs; zero coefficients are skipped at evaluation.
        Vec<(Rational, Atom)>,
    ),
    /// Add a rational offset to a rationally scaled product of sine/cosine factors.
    Trig {
        /// Exact additive coordinate translation.
        offset: Rational,
        /// Exact multiplicative starting value; nonnegativity is not checked.
        radius: Rational,
        /// Ordered radian angles and selectors: true means sine, false means cosine.
        factors: Vec<(Rational, bool)>,
    },
}

impl Term {
    /// Clone a Ready scalar or evaluate this recipe using the supplied local atoms.
    ///
    /// Sum skips zero coefficients and adds remaining terms in stored order, returning
    /// zero if none remain. Trig multiplies radius by each sine/cosine in radians,
    /// then adds offset; empty factors yield offset+radius. No approximation is
    /// requested explicitly and no full cache reset, validation, or budget is performed.
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

/// Construct the exact rational n·2^exponent by shifting numerator or denominator.
///
/// Negative exponents use a positive power-of-two denominator. Requires exponent
/// not equal to i64::MIN to negate it safely; no shift/allocation bound is enforced.
fn dyadic(n: &Int, exponent: i64) -> Rational {
    if exponent >= 0 {
        Rational::from_bigint(n << exponent as u64)
    } else {
        Rational::from_bigint_fraction(n.clone(), BigUint::one() << (-exponent) as u64)
            .expect("nonzero denominator")
    }
}

/// Construct n/|d| exactly, panicking if d is zero.
///
/// The denominator sign is discarded, so negative d is not interpreted as the signed
/// quotient n/d. All intended fixture denominator callers supply positive values.
fn ratio(n: &Int, d: &Int) -> Rational {
    Rational::from_bigint_fraction(n.clone(), d.magnitude().clone()).expect("nonzero denominator")
}

/// Wrap an arbitrary-size integer as an exact rational Real without float conversion.
fn int_real(n: &Int) -> Real {
    Real::new(Rational::from_bigint(n.clone()))
}

/// Scale a binary64 value by 2^e using repeated exponent chunks of magnitude 500.
///
/// Multiplication can overflow to infinity or underflow/round; this is not a checked
/// exact-scaling API. The number of iterations grows with |e| and has no work cap.
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

/// Representation-specific lattice scale/translation recipe and dimensional conversion helpers.
///
/// Widening first maps n to Wn+T with one positive integer W and three-axis shift.
/// F64 then scales by 2^shift; Integer retains that integer; Rational applies
/// (p/q times the widened coordinate plus tau/tau_den) times 2^shift. Symbolic
/// coordinates use (widened coordinate times scale atom plus tau times translation
/// atom) times 2^shift. Trig kinds are constructed separately.
///
/// For admitted non-overflowing values these are positive similarities, but stored
/// bit/magnitude metadata are nominal rather than validated bounds on every input.
/// Different scalar/length methods have different binary64 checks; they do not
/// collectively provide a general checked transform API.
struct Frame {
    /// Recipe controlling coordinate construction and scale/offset conventions.
    kind: Kind,
    /// Nominal lattice bit width used to compute shift, not a verified maximum.
    bits: u32,
    /// Nominal magnitude exponent; ignored by Integer coordinates.
    exponent: i32,
    /// Optional positive integer factor and three-axis lattice translation.
    widen: Option<(Int, [Int; 3])>,
    /// Positive odd 24-bit numerator of Rational scale.
    p: Int,
    /// Positive odd 24-bit denominator of Rational scale.
    q: Int,
    /// Signed per-axis extra translations used by rational/symbolic kinds.
    tau: [Int; 3],
    /// Positive odd 16-bit translation denominators used by Rational.
    tau_den: [Int; 3],
}

impl Frame {
    /// Create nominal base-bit magnitude metadata and random positive rational/translation parameters.
    ///
    /// Always draws p,q,tau,tau_den even for kinds that do not use them. Does not
    /// validate kind suitability, base width, exponent range, or future coordinates.
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

    /// Add a positive odd integer scale and three-axis translation to enlarge lattice coordinates.
    ///
    /// Panics for F64. Factor bits are max(bits-base,2) using saturating subtraction;
    /// requested bits below base still enlarge the lattice. Metadata therefore is not
    /// an enforced output-width bound. Remaining parameters come from new(kind,bits,0).
    fn widened(kind: Kind, base: u32, bits: u32, rng: &mut Rng) -> Self {
        assert!(kind != Kind::F64, "f64 coordinates cannot be widened");
        let factor = odd(rng, bits.saturating_sub(base).max(2));
        let shift = rand_vec(rng, bits);
        Self {
            widen: Some((factor, shift)),
            ..Self::new(kind, bits, 0, rng)
        }
    }

    /// Apply the optional integer widening scale/translation, otherwise clone n.
    ///
    /// With widening, axis must be below three or indexing panics; without widening
    /// the axis is unused. No coordinate-size check occurs.
    fn lattice(&self, axis: usize, n: &Int) -> Int {
        match &self.widen {
            Some((p, t)) => p * n + &t[axis],
            None => n.clone(),
        }
    }

    /// Return exponent minus nominal bit width in i64, without constraining the resulting power of two.
    fn shift(&self) -> i64 {
        i64::from(self.exponent) - i64::from(self.bits)
    }

    /// Convert a widened lattice coordinate to binary64 and apply the frame's dyadic shift.
    ///
    /// Asserts magnitude bit length≤53 and shift≥-1074 before ldexp. Does not check
    /// upper overflow or final finiteness; coord subsequently rejects nonfinite imports.
    /// Any widening translation inherits lattice's three-axis bound.
    fn f64(&self, axis: usize, n: &Int) -> f64 {
        let n = self.lattice(axis, n);
        assert!(n.bits() <= 53, "f64 lattice value exceeds 53 bits");
        assert!(self.shift() >= -1074, "f64 lattice value underflows");
        ldexp(n.to_f64().expect("53-bit value"), self.shift())
    }

    /// Return (sqrt(2),sqrt(3)) for Algebraic or (pi,e) for Transcendental; panic for other kinds.
    fn atoms(&self) -> (Atom, Atom) {
        match self.kind {
            Kind::Algebraic => (Atom::Sqrt2, Atom::Sqrt3),
            Kind::Transcendental => (Atom::Pi, Atom::E),
            _ => unreachable!("not a symbolic kind"),
        }
    }

    /// Build one translated coordinate as a Ready value or a two-atom sum.
    ///
    /// Integer ignores dyadic exponent metadata. Rational and symbolic recipes index
    /// three-axis translation arrays; all widened recipes also require axis<3. F64
    /// expects finite import after its bounds checks. Trig/TrigStrict panic because
    /// their points are built directly. This method does not verify requested magnitude.
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

    /// Clone the integer widening scale, or return one if no widening was configured.
    fn widen_factor(&self) -> Int {
        self.widen
            .as_ref()
            .map_or_else(Int::one, |(p, _)| p.clone())
    }

    /// Apply coordinate scale to a difference component, omitting all translations.
    ///
    /// F64 asserts ≤53 magnitude bits and imports ldexp's result, but has no lower
    /// exponent guard: underflow can become zero and nonfinite values panic on import.
    /// Integer ignores the dyadic exponent; rational and symbolic recipes retain exact
    /// scale definitions. Trig kinds reach atoms and panic.
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

    /// Scale a supplied squared-distance value by the square of all coordinate scales.
    ///
    /// Uses exact rational dyadics even for F64, with no binary64 width/finiteness
    /// restriction here. Algebraic scale contributes a factor two; Transcendental
    /// contributes PiSquared. The fallback also maps trig kinds to PiSquared, although
    /// those are not admitted framed inputs. Does not validate nonnegativity.
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

    /// Keep an integer coefficient unscaled, for use with transformed plane offsets.
    ///
    /// F64 asserts ≤53 magnitude bits and imports the integer exactly; all other
    /// kinds wrap an exact integer without consulting translations or symbolic atoms.
    fn plain(&self, n: &Int) -> Term {
        match self.kind {
            Kind::F64 => {
                assert!(n.bits() <= 53);
                Term::Ready(Real::try_from(n.to_f64().unwrap()).unwrap())
            }
            _ => Term::Ready(int_real(n)),
        }
    }

    /// Transform a plane offset consistently with unscaled normal coefficients.
    ///
    /// Widening changes offset to W·offset-normal·T. Subsequent scaling and subtraction
    /// of normal paired with the kind's translation produce the new offset. F64 uses
    /// an exact dyadic rational without the coordinate f64 width guard. Trig kinds
    /// panic through atoms; no normal nonzero or finite-output validation is added.
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

    /// Flatten each lattice point in coordinate order and apply coord to every component.
    ///
    /// The const dimension is not independently restricted. Translated or widened
    /// recipes index only three axes and can panic when D>3.
    fn points<const D: usize>(&self, points: &[[Int; D]]) -> Vec<Term> {
        points
            .iter()
            .flat_map(|p| p.iter().enumerate().map(|(axis, n)| self.coord(axis, n)))
            .collect()
    }

    /// Return the three plain normal coefficients followed by the transformed offset.
    fn plane(&self, normal: &[Int; 3], offset: &Int) -> Vec<Term> {
        let mut out: Vec<Term> = normal.iter().map(|n| self.plain(n)).collect();
        out.push(self.offset(normal, offset));
        out
    }
}

/// Draw a rational radian angle m/256 separated by more than eight numerator units from used.
///
/// Appends the accepted numerator. Callers use small valid inclusive lo..=hi ranges
/// with enough unused space. Impossible requests loop indefinitely, and arbitrary
/// extreme ranges are not guarded against subtraction overflow. Draws inherit the
/// RNG's remainder reduction rather than a proven uniform distribution.
fn angle(rng: &mut Rng, lo: i64, hi: i64, used: &mut Vec<i64>) -> Rational {
    loop {
        let m = rng.range(lo, hi);
        if used.iter().all(|u| (u - m).abs() > 8) {
            used.push(m);
            return Rational::fraction(m, 256).unwrap();
        }
    }
}

/// Return one for None, or 1+m·2^-(k+7) for a positive eight-bit m.
///
/// The finite relative increase lies from 2^-k through just below 2^(1-k), and is
/// always positive. This is a radial multiplier, not a signed perturbation.
fn bump(rng: &mut Rng, k: Option<u32>) -> Rational {
    match k {
        None => Rational::one(),
        Some(k) => &Rational::one() + &dyadic(&big(rng, 8), -(i64::from(k) + 7)),
    }
}

/// Build eight scalar terms for three circle sites and one radially perturbed query.
///
/// Center coordinates are signed dyadics; radius lies in [3/2,2). Distinct rational
/// radian angles use numerator range 0..=1600 with separation greater than eight.
/// None places the fourth point on the same mathematical circle, involving the
/// sin²+cos² identity; it does not prescribe which policy can certify that identity.
/// Finite k applies bump only to the query. Trig expressions are built on consumption.
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

/// Build fifteen terms for four sphere sites and one radially perturbed query.
///
/// Uses (cos(phi)cos(theta),cos(phi)sin(theta),sin(phi)), with separate separated
/// rational radian angle lists. Only the fifth radius receives bump for finite k;
/// None puts all five on the mathematical sphere. No independent noncoplanarity
/// check is applied to the first four sites. Term evaluation happens on consumption.
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

/// Ordered points, explicit planes, and leftover scalars decoded from one flat input.
struct Geo<P> {
    /// Points decoded first, in supplied order.
    p: Vec<P>,
    /// Explicit planes decoded next as normal plus offset.
    planes: Vec<Plane3>,
    /// All remaining scalars, interpreted by the consumer callback.
    s: Vec<Real>,
}

/// Consume the next fixed-arity point coordinates from the flat geometry stream.
trait FromReals: Sized {
    /// Consume the required coordinate count, panicking when the iterator is too short.
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self;
}

impl FromReals for Point2 {
    /// Consume x then y; panics if fewer than two scalars remain.
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self {
        Point2::new(it.next().unwrap(), it.next().unwrap())
    }
}

impl FromReals for Point3 {
    /// Consume x,y,z in order; panics if fewer than three scalars remain.
    fn take(it: &mut std::vec::IntoIter<Real>) -> Self {
        Point3::new(it.next().unwrap(), it.next().unwrap(), it.next().unwrap())
    }
}

/// Consume a flat scalar vector as points, then normal/offset plane groups, then leftovers.
///
/// Each point consumes its FromReals arity; each plane consumes four scalars.
/// Insufficient input panics through unwrap. Extra scalars are deliberately retained
/// in s rather than rejected; no geometric validity or total-length check occurs.
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

/// Geometry holder that either retains a built object or reconstructs scalar recipes per call.
///
/// Readiness is determined by the terms, not by the kind label. Any non-Ready term
/// keeps the whole recipe for reconstruction, including cloned Ready values. Fresh
/// local atom cells do not establish globally cold caches or eliminate shared state.
/// No constructor/result validation or resource budget is added by this holder.
struct Input<G> {
    /// Selected policy passed unchanged to the callback.
    policy: PredicatePolicy,
    /// Retained geometry when every original term was Ready.
    warm: Option<G>,
    /// Recipes retained only when at least one scalar needs reconstruction.
    terms: Vec<Term>,
    /// Decoder invoked at construction for ready inputs or per run for recipes.
    build: fn(Vec<Real>) -> G,
}

impl<G> Input<G> {
    /// Choose retained geometry when every term is Ready; otherwise store all terms for reconstruction.
    ///
    /// For the ready path consumes scalar values and calls build immediately, then
    /// stores an empty terms vector. Empty input also follows this path. The other
    /// path defers build to run. Kind chooses policy only; builder panics propagate.
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

    /// Invoke f with retained geometry, or rebuild with fresh local atoms then invoke it.
    ///
    /// Reconstruction, builder work, and destruction of temporary geometry occur during
    /// this call. Ready terms in a mixed recipe are cloned and can share scalar caches.
    /// Returns f's result unchanged; does not inspect success, certainty, or correctness.
    fn run<R>(&self, f: impl FnOnce(&G, PredicatePolicy) -> R) -> R {
        if let Some(g) = &self.warm {
            return f(g, self.policy);
        }
        let atoms = Atoms::default();
        let g = (self.build)(self.terms.iter().map(|t| t.real(&atoms)).collect());
        f(&g, self.policy)
    }
}

/// Planar geometry plus policy and retention/reconstruction strategy for timed callbacks.
type In2 = Input<Geo<Point2>>;
/// Spatial geometry plus policy and retention/reconstruction strategy for timed callbacks.
type In3 = Input<Geo<Point3>>;

// ---------------------------------------------------------------------------
// The four determinant predicates.

/// Nominal planar orientation lattice width; actual f64 coordinates still pass Frame width assertions.
const ORIENT2_BITS: u32 = 48;
/// Nominal spatial orientation lattice width before unimodular mixing and conversion checks.
const ORIENT3_BITS: u32 = 44;
/// Circle frame scaling width; the generator separately chooses its fixed center/rotation widths.
const CIRCLE_BITS: u32 = 52;
/// Sphere frame scaling width; the generator separately chooses quaternion and center widths.
const SPHERE_BITS: u32 = 52;

/// Callback producing ordered D-dimensional sites for a finite level or exact-degeneracy sentinel None.
type Lattice<const D: usize> = fn(&mut Rng, Option<u32>) -> Vec<[Int; D]>;

/// One named determinant workload's dimensional generator, decoder metadata, and value-only callbacks.
///
/// The private registry is trusted: names choose decoders, generator preference is
/// planar-first, and inactive callbacks panic. This record validates none of those
/// relationships and carries no independent expected sign or certainty assertion.
struct Det {
    /// Exact dispatch/report name; also selects decoder point count and specialized registration paths.
    name: &'static str,
    /// Human-readable operation description interpolated into emitted atlas descriptions.
    what: &'static str,
    /// Nominal lattice bit width supplied to Frame; not an enforced coordinate bound.
    base: u32,
    /// Family finite-level limit, further capped by standard axis/fuzz helpers.
    kmax: u32,
    /// Optional planar generator, preferred when present even if make3 is also present.
    make2: Option<Lattice<2>>,
    /// Optional spatial generator used only when make2 is absent.
    make3: Option<Lattice<3>>,
    /// Planar callback returning a decided sign or None, without the original certainty/reason.
    run2: fn(&Geo<Point2>, PredicatePolicy) -> Option<hyperlimit::Sign>,
    /// Spatial callback returning a decided sign or None, without the original certainty/reason.
    run3: fn(&Geo<Point3>, PredicatePolicy) -> Option<hyperlimit::Sign>,
}

/// Build three 48-bit-base planar determinant sites and discard the returned source direction.
///
/// Inherits near_collinear2's exact determinant/degenerate branches, input
/// preconditions, and unbounded coprime search; no predicate result is checked here.
fn orient2_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 2]> {
    near_collinear2(rng, ORIENT2_BITS, k).0.to_vec()
}
/// Build the Generic 44-bit-base tetrahedron and retain only its four sites.
///
/// Inherits near_coplanar3's determinant construction and search preconditions;
/// apex and in-plane auxiliary displacements are discarded.
fn orient3_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 3]> {
    near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Generic)
        .points
        .to_vec()
}
/// Return three circle sites and query from the exact lattice-circle generator.
///
/// Discards center/radius metadata and inherits the unbounded candidate search;
/// this wrapper does not independently test radial relations or predicate signs.
fn incircle_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 2]> {
    near_cocircular2(rng, k).points.to_vec()
}
/// Return four sphere sites and query from the lattice-sphere generator.
///
/// Discards center/radius metadata and inherits noncoplanarity/decomposition searches;
/// no independent predicate or receipt assertion occurs.
fn insphere_points(rng: &mut Rng, k: Option<u32>) -> Vec<[Int; 3]> {
    near_cospherical3(rng, k).points.to_vec()
}

/// Panic if an inactive planar callback is invoked; a Det record must select the spatial generator instead.
fn no2(_: &Geo<Point2>, _: PredicatePolicy) -> Option<hyperlimit::Sign> {
    unreachable!()
}
/// Panic if an inactive spatial callback is invoked; a Det record must select the planar generator instead.
fn no3(_: &Geo<Point3>, _: PredicatePolicy) -> Option<hyperlimit::Sign> {
    unreachable!()
}

/// Return the four determinant registry entries in orientation/circle/sphere order.
///
/// Each selects exactly one dimensional generator; the other callback panics.
/// Callbacks index the required point count and return only the outcome value,
/// dropping certainty and Unknown reasons. kmax is 96,132,92,92 respectively;
/// standard finite axes later cap it at 96. No record-consistency validation occurs.
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
    /// Register a dimensional sweep with lattice level k(m) and the supplied frame constructor.
    ///
    /// Prefers make2 if both generators are present; does nothing if neither exists.
    /// Builds sites before drawing frame parameters. Atlas seeds by operation and label,
    /// regenerates inputs outside timing by default, then times Input::run; symbolic
    /// reconstruction happens inside that call. Option presence marks only the initial
    /// probe's success, without a sign/receipt assertion. Builder/callback consistency
    /// and valid levels are caller obligations; no local resource limit is imposed.
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

    /// Choose the three-point decoder only for the exact name orient2, otherwise the four-point decoder.
    ///
    /// Does not check make2 or reject other names; registry consistency is assumed.
    fn build2(&self) -> fn(Vec<Real>) -> Geo<Point2> {
        if self.name == "orient2" {
            geo2_3
        } else {
            geo2_4
        }
    }

    /// Choose the four-point decoder only for the exact name orient3, otherwise the five-point decoder.
    ///
    /// Does not check make3 or reject other names; registry consistency is assumed.
    fn build3(&self) -> fn(Vec<Real>) -> Geo<Point3> {
        if self.name == "orient3" {
            geo3_4
        } else {
            geo3_5
        }
    }

    /// Register degeneracy versus a second frame axis using the selected dimensional generator.
    ///
    /// Prefers make2; neither generator means no registration. Sites are generated
    /// before frame parameters. Atlas seeds separately from ordinary sweeps and may
    /// block later cells based on observed/predicted cost; this is not guaranteed full
    /// grid completion. Callbacks retain values only and no expected result is asserted.
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

    /// Register seeded randomized atlas samples, jittering finite levels upward by 0..=7.
    ///
    /// Clamps finite k to min(kmax,96); values at least 112 select exact degeneracy.
    /// This differs from level for NaN, which follows the finite cast path here.
    /// Intended axes are bounded nonnegative values; cast/add arithmetic is not general
    /// input validation. Draws k, then lattice sites, then frame parameters in that order.
    /// Dimensional preference and unvalidated callback assumptions match sweep. The
    /// shared harness can stop after ceiling/panic observations; this is not libFuzzer.
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

    /// Register robust's adaptive binary64 determinant using this family's lattice/frame recipe.
    ///
    /// Dispatches only the four known names, panicking otherwise. It constructs the
    /// frame before sites, unlike the Hyperlimit sweep; the reference label also selects
    /// a different RNG stream. Thus it does not run on the exact same sampled inputs.
    /// Raw numeric outcomes always count as success in the shared atlas, without a
    /// cross-library sign comparison, finiteness check, or certainty receipt.
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

    /// Register approximate and strict trigonometric circle/sphere sweeps; ignore other names.
    ///
    /// Source sites use rational-angle sine/cosine identities. Both series rebuild terms
    /// within the timed callback, but their labels yield different sampled inputs.
    /// Only value presence is recorded; the method does not assert which policy decides
    /// an identity, whether a decision is approximate, or agreement between series.
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

    /// Register direct and prepared-evidence series for f64 and rational inputs.
    ///
    /// Prepared builders derive line orientation, plane evidence, or lifted-circle/sphere
    /// evidence outside the measured callback. Orientation preparation is consumed by
    /// line/plane sidedness APIs, so its output type differs from the direct Sign result.
    /// Both sides retain value presence only and never assert corresponding results.
    ///
    /// Direct and prepared labels select different RNG streams; they are not paired
    /// queries on identical source geometry. Under the default harness, each timed call
    /// receives regenerated ready geometry/evidence rather than repeatedly reusing one
    /// object. Unknown names panic in prepared dispatch; malformed registry data or
    /// fixture generation can also panic or run without a per-call timeout.
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

/// Decode three planar points, no planes, and any leftover scalars; panic if fewer than six values remain.
fn geo2_3(v: Vec<Real>) -> Geo<Point2> {
    geo(v, 3, 0)
}
/// Decode four planar points, no planes, and leftover scalars; panic if fewer than eight values remain.
fn geo2_4(v: Vec<Real>) -> Geo<Point2> {
    geo(v, 4, 0)
}
/// Decode four spatial points, no planes, and leftover scalars; panic if fewer than twelve values remain.
fn geo3_4(v: Vec<Real>) -> Geo<Point3> {
    geo(v, 4, 0)
}
/// Decode five spatial points, no planes, and leftover scalars; panic if fewer than fifteen values remain.
fn geo3_5(v: Vec<Real>) -> Geo<Point3> {
    geo(v, 5, 0)
}

/// Apply Frame::f64 coordinatewise to planar lattice points for reference predicates.
///
/// Preserves point/coordinate order and inherits width/lower-shift assertions.
/// Does not import to Real or independently reject a nonfinite conversion result.
fn raw2(frame: &Frame, pts: &[[Int; 2]]) -> Vec<[f64; 2]> {
    pts.iter()
        .map(|p| [frame.f64(0, &p[0]), frame.f64(1, &p[1])])
        .collect()
}
/// Apply Frame::f64 in x,y,z order to every spatial lattice point.
///
/// Inherits the frame's coordinate assertions and unchecked upper-overflow behavior;
/// returns raw binary64 arrays without predicate or finiteness validation.
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
/// Copy a binary64 pair into robust::Coord without arithmetic or validation.
fn c2(p: [f64; 2]) -> robust::Coord<f64> {
    robust::Coord { x: p[0], y: p[1] }
}
/// Copy a binary64 triple into robust::Coord3D without arithmetic or validation.
fn c3(p: [f64; 3]) -> robust::Coord3D<f64> {
    robust::Coord3D {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}

/// Register four determinant families across degeneracy, preparation, exponent, and bit-width workloads.
///
/// For each family, nominal degeneracy sweeps five framed kinds plus the robust
/// reference; circle/sphere also add approximate and strict trig recipes. Fuzz
/// samples vary finite k within an eight-level band and use f64 exponents -300..=300
/// or rational widths 64..=1024. This is seeded sampling, not a cargo-fuzz target.
/// Grids combine degeneracy with f64 exponents or integer/transcendental widths.
///
/// Exponent sweeps request -1000..=960 in steps of 40; grids request -1000..=1000
/// in steps of 125. Bit sweeps use thirteen sizes from 64 through 4096, integer
/// grids use powers 64..4096, and transcendental grids stop at 512. Prepared/direct
/// series use separate label-derived RNG streams and do not compare identical inputs
/// or assert equivalent outputs. Integer framing ignores magnitude scaling, so the
/// shared near-one prose does not apply to its raw lattice coordinates.
///
/// The emitted description strings retain qualitative timing/refusal claims such as
/// a cliff near k≈50 and STRICT refusing trig degeneracy. This registration does
/// not measure/assert those claims or certify the receipts behind decided values.
/// Atlas may stop or skip later samples; None/error statuses do not preserve all
/// kernel reasons, and success flags inspect only the initial probe. No complete
/// geometric qualification or uniform practical bound follows from registration.
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

/// Infer the smallest positive d with d²+d≥input length and partition into d+1 rows of at most d scalars.
///
/// Intended inputs contain exactly d(d+1) entries. No equality or shape validation
/// occurs: short/malformed input produces ragged or empty rows, and empty input
/// returns two empty rows. Dimension search and allocations have no separate bound.
fn nd_build(v: Vec<Real>) -> Vec<Vec<Real>> {
    // (d + 1) points of dimension d: d^2 + d terms.
    let d = (1..).find(|d| d * d + d >= v.len()).unwrap();
    let mut it = v.into_iter();
    (0..=d).map(|_| it.by_ref().take(d).collect()).collect()
}

/// Infer the smallest positive d with d²+2d≥input length, then decode simplex rows and a leftover query.
///
/// Consumes up to d values for each of d+1 rows; all leftovers form the query.
/// The intended exact count is d(d+2), but malformed/short input yields ragged rows
/// or a short/empty query without validation. Empty input gives two empty simplex
/// rows and an empty query. No independent dimension/allocation budget is enforced.
fn nd_sphere_build(v: Vec<Real>) -> (Vec<Vec<Real>>, Vec<Real>) {
    // (d + 1) simplex points and a query: d^2 + 2d terms.
    let d = (1..).find(|d| d * d + 2 * d >= v.len()).unwrap();
    let mut it = v.into_iter();
    let simplex = (0..=d).map(|_| it.by_ref().take(d).collect()).collect();
    (simplex, it.collect())
}

/// Build d random 44-bit coordinate rows and a final row controlling affine dependence.
///
/// None adds another independently generated row. Some(delta) forms an integer
/// affine combination of the first d rows, then adds delta to the last coordinate.
/// Delta zero therefore gives an exact affine dependency; nonzero delta does not
/// prove nondegeneracy if the base is dependent or that axis lies in its affine
/// span. Some requires d≥1 or indexing panics; None permits d=0 and returns one
/// empty point. Coefficient accumulation and allocation are not separately bounded.
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

/// Flatten arbitrary-dimensional lattice rows through one frame, reusing axis two for all later axes.
///
/// Each coordinate at index≥2 shares the third translation parameters. This still
/// applies a common scale and a fixed translation vector, but does not supply
/// independent random translations in every dimension. Row shape is not validated.
fn nd_terms(frame: &Frame, pts: &[Vec<Int>]) -> Vec<Term> {
    pts.iter()
        .flat_map(|p| {
            p.iter()
                .enumerate()
                .map(|(axis, n)| frame.coord(axis.min(2), n))
        })
        .collect()
}

/// Sweep dimensions 2 through 9 for seven orientation and seven in-sphere recipe series.
///
/// Orientation rows cover generic, exact affine dependence, and a one-unit last-axis
/// perturbation of 44-bit bases. An extra f64 series returns affine_independent_d's
/// `Option<bool>`; Some(false) counts as atlas success. Every callback discards certainty
/// and Unknown reasons, and no expected sign or independence value is asserted.
///
/// Sphere fixtures use a 44-bit center and positive odd 40-bit radius with a full
/// axis simplex. Queries lie inside at the negative half-radius rounded toward the
/// center, exactly on the sphere, or one unit outside. The reused near-series label
/// k≈44 describes the orientation base, not this radius-relative sphere gap, which
/// is on the scale of 2^-40. Frame inputs are built outside the timer; symbolic
/// terms are rebuilt by Input::run during timing. Atlas's sampling/stopping rules
/// can omit later dimensions and do not enforce a timeout within a call.
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

/// Register a framed sweep for each kind, preserving the composite fixture's scalar order.
///
/// For each axis value, calls make with level(value), then creates an unwidened
/// zero-exponent Frame with the supplied nominal base width. Serializes points,
/// then each plane's normal and offset, then squared quantities, then lengths.
/// The build callback must decode exactly that ordering; no dimension, count,
/// nonnegativity, or geometric-precondition validation is added here.
///
/// Atlas regenerates Input outside each timed call with its default settings;
/// Input::run rebuilds symbolic terms inside timing and selects the kind's policy.
/// The callback's Outcome governs the initial success probe, without any separate
/// expected-result or certainty assertion. Inherits frame/conversion panics and
/// unbounded fixture searches; the axis does not impose a total work bound.
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

/// Unvalidated integer-lattice ingredients serialized by derived for composite fixtures.
///
/// Separates quantities by their transformation law: translated points, planes,
/// squared distances, and linearly scaled differences. This storage is not a
/// validated region, geometric result, or proof of any predicate precondition.
struct Shape<const D: usize> {
    /// Ordered D-dimensional sites; Frame applies its common coordinate scale and translation.
    points: Vec<[Int; D]>,
    /// Spatial normals and offsets for n·x+offset=0, independent of the point dimension D.
    planes: Vec<([Int; 3], Int)>,
    /// Squared quantities scaled by the square of the frame scale; nonnegativity is not enforced.
    squared: Vec<Int>,
    /// Linearly scaled scalars, including radii or signed direction components, without translation.
    lengths: Vec<Int>,
}

impl<const D: usize> Shape<D> {
    /// Retain the supplied sites and initialize all non-point ingredients as empty vectors.
    ///
    /// Accepts empty or degenerate point lists without validating dimension or geometry.
    fn points(points: Vec<[Int; D]>) -> Self {
        Self {
            points,
            planes: Vec::new(),
            squared: Vec::new(),
            lengths: Vec::new(),
        }
    }
}

/// Rotate an integer planar vector left by 90 degrees without division or rounding.
///
/// Returns (-y,x), preserving squared norm and giving zero dot product with the
/// input. Zero remains zero; callers must establish nonzero directions themselves.
fn perp(u: &[Int; 2]) -> [Int; 2] {
    [-u[1].clone(), u[0].clone()]
}

/// Register six planar composite-predicate workloads using the four KINDS frame recipes.
///
/// The standard axis has finite levels 0,4,...,96 and the degenerate sentinel.
/// Point/segment, segment/segment, and point/triangle rows reuse near_collinear2:
/// with sites a,b,c and u=b-a, they query c against ab, intersect ab with
/// `[c,c+perp(u)]`, or query c against triangle `[a,b,a+perp(u)]`. At the sentinel,
/// c lies strictly between a and b, giving on-segment, T-junction, and on-edge
/// geometry respectively. Finite levels control a determinant, not a guaranteed
/// interior Euclidean projection or distance bound as the report prose suggests.
///
/// Distance ordering compares a circle site's and the generated query's squared
/// distances to the center. Its axis drops level 96, keeping 0..92 by fours plus
/// the sentinel. Circle/line and circle/segment rows use endpoints site±perp(w),
/// where w=site-center, and pass the original or perturbed squared radius through
/// Shape::squared. The unperturbed segment contains the exact tangent point.
/// These rows still generate and discard an extra perturbed circle query, inheriting
/// its unbounded decomposition search even when the requested radius is unperturbed.
///
/// All callbacks take only the predicate value, discarding certainty and Unknown
/// reasons. Some counts as atlas success without checking the relation; no exact
/// expectation or replay assertion is made. Generation, conversion, and timing
/// limitations are inherited from derived and the source lattice helpers.
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

/// Register spatial incidence, distance, intersection, homogeneous, and box workloads.
///
/// Uses KINDS (no Integer recipe), ordinarily levels 0..96 by fours plus the
/// sentinel; point/plane and homogeneous/plane stop at 72, sphere distance ordering
/// at 92, and sphere intersection, lexicographic ordering, and two box rows at 44.
/// The plane/box row also uses 72. These are nominal input axes, not certified
/// Euclidean proximity bounds or guarantees that all samples complete.
///
/// Point/plane uses a 24-bit normal with a controlled plane expression, whose finite
/// nonzero residual bottoms out at magnitude two. Triangle, tetrahedron, segment,
/// ray, and triangle-pair rows reuse near_coplanar3's Inside construction and sites
/// `[a,b,c,d,e,f]`, with e=d+apex_offset and f=e+in_plane. They query d against abc
/// or abce, de against abc, the ray (d,e-d) against abc, or abc against def.
/// The ray direction is scaled through Shape::lengths without translation and
/// rebuilt as a Point3 inside the timed callback. Positive oriented normal pairing
/// does not imply that the ray points away from d's side of the plane. Source lifts
/// and determinant-one shears prove neither orthogonal interior projection nor a
/// well-conditioned tetrahedron. The separate segment/segment row uses Crossing,
/// whose sentinel yields a proper crossing; finite lifts need not intersect.
///
/// Sphere distance ordering compares an on-sphere site with the generated query.
/// Point/sphere perturbs the squared radius while retaining an on-sphere site;
/// sphere/sphere uses ordinary radii R/3 and R-R/3, perturbing the second by a
/// relative_delta at k+2. R is divisible by three, so their unperturbed sum equals
/// the center separation R. The called API classifies closed balls using ordinary
/// radii; it does not classify surface-sphere intersections. These last two rows
/// still generate an unused perturbed query and inherit its unbounded search.
///
/// Point/segment distance uses endpoints a,a+2h and query a+h+h×r. Its projection
/// is the exact segment midpoint and its squared distance is |h×r|². Parallel h,r
/// are not rejected: zero distance can receive a negative perturbed threshold.
/// Point/plane distance uses a nonzero integer-length normal and offset chosen for
/// a positive 40-bit distance, then perturbs its square. Triangle degeneracy and
/// point/segment location reuse near_collinear3; its finite levels likewise do not
/// guarantee interior projection. Lexicographic queries differ only in z, or are
/// identical at the sentinel.
///
/// The homogeneous row builds three planes through one lattice point and a fourth
/// with a controlled residual. It never rejects dependent first-three normals, so
/// their minors can be the all-zero carrier. The callback tests Boolean incidence,
/// not the sidedness asserted by the report string, and Some(false) still counts as
/// success. Ready-input construction occurs outside timing; symbolic construction
/// occurs inside Input::run. No validity or unique-intersection check is performed.
///
/// Box fixtures have positive extents, a signed x gap, overlapping y intervals,
/// and identical z intervals. Their point query is the second box's minimum,
/// always on the first box's minimum-z plane: a negative x gap gives boundary
/// membership, not strict interior. The plane/box fixture grows from the selected
/// corner along coordinate signs that increase the magnitude of the plane residual;
/// its sentinel gives corner contact. Nonzero residual is not unit-normal distance.
///
/// Every predicate callback discards receipts and Unknown reasons through value().
/// The atlas checks Option presence on its initial probe without asserting any
/// classification, validating reports, or replaying constructions. Unbounded source
/// searches, frame width assertions, and shared timing limitations still apply.
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

/// Lift the controlled planar determinant fixture into the plane z=alpha*x+beta*y.
///
/// Draws alpha,beta in -3..=3, then calls near_collinear2 with ORIENT2_BITS.
/// If its determinant is delta, the spatial cross product is
/// (-alpha*delta,-beta*delta,delta), so collinearity is preserved exactly. None
/// keeps the third point strictly between the first two. This injective linear lift
/// is not generally an isometry, and finite levels do not prove interior projection.
/// Inherits the planar generator's level preconditions and unbounded coprime search.
fn near_collinear3(rng: &mut Rng, k: Option<u32>) -> [[Int; 3]; 3] {
    let (alpha, beta) = (rng.range(-3, 3), rng.range(-3, 3));
    let (points, _) = near_collinear2(rng, ORIENT2_BITS, k);
    points.map(|[x, y]| {
        let z = &x * alpha + &y * beta;
        [x, y, z]
    })
}

/// Extend the Inside tetrahedron fixture to six sites for triangle and ray queries.
///
/// Preserves `[a,b,c,d]`, appends e=d+apex_offset and f=e+in_plane, and leaves all
/// non-point Shape vectors empty. The apex displacement has positive oriented
/// normal pairing; it is not generally perpendicular after the source shear and
/// need not point away from d's side. No intersection, conditioning, or expected
/// predicate value is checked. Uses the same recipe as composite_3d's local closure.
fn with_apex_shape(rng: &mut Rng, k: Option<u32>) -> Shape<3> {
    let t = near_coplanar3(rng, ORIENT3_BITS, k, Coplanar::Inside);
    let [a, b, c, d] = t.points;
    let e = add(&d, &t.apex_offset);
    let f = add(&e, &t.in_plane);
    Shape::points(vec![a, b, c, d, e, f])
}

/// Construct a nonzero integer normal together with its exact positive integer length.
///
/// Chooses a radius-nine lattice vector from sphere_9 (including axis points, not
/// only permutations of (1,4,8)) and four signed 8-bit quaternion components.
/// Writing Q for their squared norm, the unnormalized quaternion matrix satisfies
/// RᵀR=Q²I. Thus n=Rv has length 9Q, returned without taking a square root.
/// The quaternion components are nonzero by construction, so Q>0. Sampling inherits the RNG's
/// remainder reduction; no uniform distribution on directions is asserted.
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

/// Choose a scalar-pair recipe and its visible policy for atlas comparisons.
///
/// These are construction recipes, not a complete inventory of Real variants.
#[derive(Clone, Copy, PartialEq)]
enum Scalar {
    /// Import stored binary64 values exactly; the preceding arithmetic used binary64 rounding.
    F64,
    /// Use the stored rational x,y directly under APPROX.
    Rational,
    /// Construct square-root expressions inside timing under APPROX.
    Sqrt,
    /// Construct the same square-root forms but require STRICT decisions.
    SqrtStrict,
    /// Construct natural-logarithm expressions inside timing under APPROX.
    Ln,
}

impl Scalar {
    /// Return the stable report label, also used to derive this series's RNG stream.
    fn label(self) -> &'static str {
        match self {
            Scalar::F64 => "f64",
            Scalar::Rational => "rational",
            Scalar::Sqrt => "sqrt",
            Scalar::SqrtStrict => "sqrt strict",
            Scalar::Ln => "ln",
        }
    }
    /// Select STRICT only for SqrtStrict; every other recipe permits the approximate terminal.
    fn policy(self) -> PredicatePolicy {
        if self == Scalar::SqrtStrict {
            STRICT
        } else {
            APPROX
        }
    }
}

/// Scalar recipes in report order, with separate approximate and strict square-root rows.
const SCALARS: [Scalar; 5] = [
    Scalar::F64,
    Scalar::Rational,
    Scalar::Sqrt,
    Scalar::SqrtStrict,
    Scalar::Ln,
];

/// Retain rational and binary64 ingredients for constructing a scalar comparison inside timing.
///
/// Nondegenerate recipes increase y relative to x or b relative to a; square root
/// and logarithm change that relative gap. At the sentinel, rational/binary64 values
/// are identical, while root/log recipes use equal-valued expression identities.
/// Equality or ordering is mathematical intent, not a promise of a decided predicate.
struct Pair {
    /// Recipe and policy selector used when materializing and comparing values.
    kind: Scalar,
    /// Exact positive rational base, generated in [3/2,2).
    x: Rational,
    /// Perturbed rational, equal copy, or independent positive factor for an equality identity.
    y: Rational,
    /// Binary64 base 1+unit; rounding permits 2 as well as values in [1,2).
    a: f64,
    /// Binary64 perturbed value, or exactly a at the equality sentinel.
    b: f64,
    /// Select root/log equality identities instead of applying each function separately.
    degenerate: bool,
}

impl Pair {
    /// Draw both rational and binary64 ingredients for the selected comparison recipe.
    ///
    /// Sets x=1+m*2^-40 for a 40-bit positive m, so x is in [3/2,2).
    /// For Some(k), y=x*(1+bump), with bump=q*2^-(k+7) and 8-bit q;
    /// a=1+unit and b=a*(1+2^-k) use rounded binary64 arithmetic instead.
    /// F64 sweeps cap k at 52; larger private requests can round b back to a.
    /// Extreme k also reaches a narrowing i32 cast and unchecked negation in powi.
    ///
    /// None clones x for Rational, otherwise draws a separate 30-bit-part y in
    /// [3/2,2); it sets b=a in every case. Even unused ingredients consume RNG
    /// values. This setup does not construct roots/logarithms or test decisions.
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

    /// Materialize this pair's Real operands, constructing roots/logarithms on each call.
    ///
    /// F64 imports a,b exactly and panics on invalid imports. Rational wraps x,y.
    /// Ordinary root/log rows apply that function to each rational independently;
    /// degenerate rows instead use sqrt(x)*sqrt(y) versus sqrt(x*y), or ln(x)+ln(y)
    /// versus ln(x*y). Positive generated x,y justify both identities and unwraps;
    /// this method does not validate arbitrarily replaced fields or certify a later
    /// comparison. Fresh expression construction does not reset shared/global caches.
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

/// Sweep scalar comparison, sign, enclosure, interval, and reciprocal recipes.
///
/// Five pair-based operations use all SCALARS, levels 0..52 for F64 or 0..96 for
/// other kinds in steps of four, followed by the equality sentinel. Pair::values
/// builds the Reals inside each timed callback. The first four callbacks return
/// whether a decision exists; the interval-sign filter returns its optional sign.
/// Neither checks the expected order/sign/location or retains the certainty receipt.
/// At exact equality the ball `[b-a,b+a]` and interval `[b-a,b]` touch zero and have
/// no uniform sign; a declined filter is recorded as a failed atlas probe.
///
/// The certified_interval_sign name and report string do not promise a certified
/// receipt: that API can consume approximation under its policy, and this callback
/// discards the qualification. Source x/y relative gaps are transformed nonlinearly
/// by square root or logarithm rather than imposing the report's exact output ratio.
///
/// Reciprocal series use rational, square-root, and ln(1+x) operands with numerator
/// and denominator bit widths 8,16,...,8192. The positive rational ratio lies strictly
/// between 1/2 and 2, but reduction can shrink its parts. Scalar construction is timed;
/// APPROX is used for every reciprocal, with errors and Unknown both mapped to None
/// and successful receipts dropped. No inverse identity or source replay is checked.
fn scalar_predicates(atlas: &mut Atlas) {
    let axis = degeneracy_axis(96);
    let f64_axis = degeneracy_axis(52);
    /// Timed scalar callback whose Boolean reports decision presence, not the expected answer.
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

/// Generate a circle-inspired lattice ring and its integer center using rounded binary64 trigonometry.
///
/// Uses a signed 44-bit center, radius 2^40*(1+unit), a random phase, and one
/// jittered angle per equal angular sector. Rounds each sine/cosine displacement to
/// an integer before adding the center. These rounded vertices are the model passed
/// to exact predicates; they are not exact analytic-circle sites or certified
/// approximations to that circle. No convexity, simplicity, or distinctness check
/// occurs. The swept sizes are 4..16384; the helper itself also accepts n=0,1,2,
/// which cannot define the advertised strictly convex polygon. Large n has no
/// allocation bound and its conversion to binary64 need not preserve every index.
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

/// Frame the generated lattice ring followed by its center as the final point.
///
/// Uses an unwidened nominal 46-bit frame and decodes all coordinate pairs into Geo.
/// The final center is the query used by polygon callbacks; n=0 still produces one
/// point. Geometry validity and trig-rounding fidelity are not checked. Ready/symbolic
/// construction and kind restrictions follow Frame and Input.
fn polygon_input(kind: Kind, n: usize, rng: &mut Rng) -> In2 {
    let (mut points, center) = convex_polygon(rng, n);
    points.push(center);
    let frame = Frame::new(kind, 46, 0, rng);
    Input::new(kind, frame.points(&points), |v| {
        let n = v.len() / 2;
        geo::<Point2>(v, n, 0)
    })
}

/// Build n closed halfspaces with a common strict interior point and exact lattice offsets.
///
/// For each signed 16-bit normal n, sets L=floor(sqrt(n·n))+1 and offset
/// -n·center-L*2^40. Thus the center's residual is strictly negative, and its
/// supporting-plane distance exceeds 2^40 before framing, including when n·n is a
/// perfect square. The shared nominal 46-bit frame preserves these inequalities.
/// No check ensures distinct planes, bounded intersection, or a particular shape;
/// n=0 yields an empty system. Serializes the center then normal/offset tuples,
/// and decodes their counts from the flat vector. Allocation has no separate bound.
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

/// Register ring, halfspace, and support-envelope size sweeps for F64, Rational, and Transcendental.
///
/// Four ring operations use powers of two from 4 to 16384 vertices. Area sign and
/// both membership rows check only decision presence; ring_convexity alone checks
/// LocallyConvex. Each row gets its own label/operation-derived input stream, not
/// one shared polygon. Rounded-lattice geometry is authoritative, and no independent
/// exact convexity, containment, area, or analytic-circle fidelity test is made.
///
/// Point/halfspace uses 4..4096 planes and feasibility uses 4..256. Both reuse
/// convex_planes, whose center proves nonempty intersection but not boundedness as
/// the feasibility report string claims. The feasibility callback ignores that known
/// center and accepts any returned report, without checking feasibility or its witness.
/// No scalar-inclusive polynomial running-time bound is established here.
///
/// Support DOP rows use 4..8192 random spatial points and thirteen fixed unnormalized
/// integer axes (three coordinate, four body-diagonal, six face-diagonal directions).
/// Axes and their Vec allocation are rebuilt inside the timed callback. Repeated
/// timing loops also include dropping the returned carrier; the initial probe stops
/// timing before that drop. Only Option presence is checked,
/// not support extrema, source incidence, or certainty. Input regeneration, symbolic
/// reconstruction, and atlas stopping limitations apply to every series.
fn polygons(atlas: &mut Atlas) {
    let sizes = powers_of_two(2, 14);
    let kinds = [Kind::F64, Kind::Rational, Kind::Transcendental];
    /// Ring/query callback reporting a decision or the explicit local-convexity expectation.
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

/// Sweep three construction paths at nominal widths 64,128,...,4096 across four non-F64 kinds.
///
/// Line/line uses four independent signed 48-bit lattice sites widened by Frame.
/// It does not reject collapsed, parallel, or coincident supports in generation;
/// the Option-returning API marks only presence as success and supplies no receipt.
/// The example does not recheck that the returned point lies on either support.
///
/// Three-plane construction generates each plane through a separately drawn point,
/// then widens their offsets consistently. No rank or homogeneous-carrier validity
/// check occurs: the result is unconditionally wrapped in Some, so all-zero or
/// infinite carriers also count as successful probes. Policy is unused in that call.
///
/// Segment/plane starts with nonzero residual t and n2=n·n, then displaces the other
/// endpoint by -sign(t)*(floor(|t|/n2)+2)*n. Its residual has the opposite strict
/// sign, proving a source crossing. The callback checks ProperCrossing only; it does
/// not inspect/replay the returned point, parameter, or construction evidence.
///
/// Integer, Rational, Algebraic, and Transcendental recipes all use their ordinary
/// APPROX policy where needed. Widening sizes are nominal, not enforced output-bit
/// bounds. Exact-input construction and symbolic reconstruction follow Input's timing
/// rules; report labels do not establish certified construction or completed closure.
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

/// Measure whole orientation and in-sphere batches for F64 and Rational inputs under APPROX.
///
/// Orientation counts are powers of two 1..16384; in-sphere counts are 1..4096.
/// Each orientation level is drawn by remainder reduction into 0..=96 and each
/// sphere level into 0..=92, without the exact-degeneracy sentinel. The report's
/// uniform wording is not a distribution proof. Source generation can search
/// without an iteration bound, and frame width assertions still apply.
///
/// Extracts Ready geometry during setup; warm here means Input retained already-built
/// values, not that predicates have run or all global caches have been reset. The
/// atlas regenerates the vector outside each call with default settings. Timed work
/// includes batch output allocation, scanning for decision presence, and dropping
/// that output. Every outcome must have a value for the initial probe to succeed;
/// values and receipts are not compared with independent expected results.
///
/// BATCH_MODES selects sequential and feature-gated parallel APIs. Different labels
/// produce separate RNG streams, so these rows do not compare identical batches or
/// prove sequential/parallel agreement. Later timing-loop results are not revalidated
/// by Atlas, and timing ceilings do not interrupt an individual call.
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

/// Run sequential then parallel batch series when parallel APIs are compiled.
#[cfg(feature = "parallel")]
const BATCH_MODES: [bool; 2] = [false, true];
/// Run only sequential batch series when the parallel feature is disabled.
#[cfg(not(feature = "parallel"))]
const BATCH_MODES: [bool; 1] = [false];

/// Run the eight atlas registration groups in source order and emit the shared JSON report.
///
/// Delegates CLI handling to Atlas::from_args: --quick, --filter, --seed, and --out
/// select the harness configuration and output, with JSON otherwise sent to stdout.
/// Unknown arguments panic; output-file write failures panic. finish writes directly
/// to the destination rather than publishing atomically. Registered series remain
/// subject to filters/stopping rules, and no total timeout encloses fixture searches.
/// Does not call reuse_inputs or run a correctness/receipt/replay verification pass.
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
