//! Performance atlas for Hypertri: exact polygon triangulation, Delaunay and
//! constrained Delaunay triangulation, and the D-dimensional oracle, across
//! input shapes, sizes, coordinate representations, coordinate bit lengths,
//! near-degenerate perturbations, hole counts and constraint counts.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypertri --all-features --example perf_atlas -- --out atlas.json`.
//!
//! Every generated input lives in an integer frame `[0, 2^20)^2` unless an
//! axis says otherwise, so kinds differ by structure rather than by scale.
//! Input kinds:
//! - uniform: distinct uniformly random integer points;
//! - clustered: tight integer clusters around random centres;
//! - lattice: a square integer lattice (every unit cell is cocircular and
//!   every row, column and diagonal is collinear);
//! - circle: exactly cocircular rational points (rational parametrisation).
use std::collections::{HashMap, HashSet};
use std::f64::consts::{PI, TAU};

use hypertri::cdt::{self, CollinearPointFacts, DelaunayTriangulation};
use hypertri::nd::{self, BistellarFlipD, DelaunayComplex, PointD, Simplex};
use hypertri::{
    Cell, CellHandle, Constraint, Point2, PolygonInput, PolygonTriangulationAlgorithm,
    PredicatePolicy, QualityPolicy, Rational, Real, TdsBoundaryPolicyD, TriangulationCertainty,
    TriangulationContext, TriangulationD, TriangulationDataStructureD, TriangulationOptions,
};
use perf_atlas::{Atlas, Rng, ladder, powers_of_two};

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);
const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);

/// Side of the integer coordinate frame.
const FRAME: i64 = 1 << 20;

const POINT_KINDS: [&str; 4] = ["uniform", "clustered", "lattice", "circle"];
const POLYGON_KINDS: [&str; 4] = ["convex (cocircular)", "star", "comb", "collinear square"];

type Xy = (Rational, Rational);

fn int(v: i64) -> Rational {
    Rational::new(v)
}

fn xy((x, y): (i64, i64)) -> Xy {
    (int(x), int(y))
}

fn lift(points: &[Xy]) -> Vec<Point2> {
    points
        .iter()
        .map(|(x, y)| Point2::new(Real::new(x.clone()), Real::new(y.clone())))
        .collect()
}

fn to_f64(points: &[(i64, i64)]) -> Vec<[f64; 2]> {
    points.iter().map(|&(x, y)| [x as f64, y as f64]).collect()
}

/// `2^k` as an exact rational, for any `k >= 0`.
fn pow2(k: u32) -> Rational {
    let mut out = int(1);
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = &out * &int(1 << step);
        left -= step;
    }
    out
}

/// A random integer with exactly `bits` bits.
fn big(bits: u32, rng: &mut Rng) -> Rational {
    rng.decimal(bits).parse().expect("decimal digits")
}

/// Distinct integer points drawn from `sample`.
fn distinct(
    n: usize,
    rng: &mut Rng,
    mut sample: impl FnMut(&mut Rng) -> (i64, i64),
) -> Vec<(i64, i64)> {
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(n);
    let mut attempts = 0;
    while out.len() < n && attempts < n * 64 + 1024 {
        attempts += 1;
        let p = sample(rng);
        if seen.insert(p) {
            out.push(p);
        }
    }
    out
}

fn uniform_ints(n: usize, rng: &mut Rng) -> Vec<(i64, i64)> {
    distinct(n, rng, |r| (r.range(0, FRAME - 1), r.range(0, FRAME - 1)))
}

fn clustered_ints(n: usize, rng: &mut Rng) -> Vec<(i64, i64)> {
    let clusters = (n / 64).max(1);
    let centres: Vec<(i64, i64)> = (0..clusters)
        .map(|_| {
            (
                rng.range(FRAME / 8, FRAME * 7 / 8),
                rng.range(FRAME / 8, FRAME * 7 / 8),
            )
        })
        .collect();
    distinct(n, rng, |r| {
        let (cx, cy) = centres[r.below(clusters as u64) as usize];
        (
            cx + r.range(-512, 512) + r.range(-512, 512),
            cy + r.range(-512, 512) + r.range(-512, 512),
        )
    })
}

/// The first `n` points of a square lattice spanning the frame, with spacing.
fn lattice_ints(n: usize) -> (Vec<(i64, i64)>, i64, usize) {
    let side = (n as f64).sqrt().ceil().max(1.0) as usize;
    let spacing = (FRAME / side as i64).max(1);
    let points = (0..n)
        .map(|i| ((i % side) as i64 * spacing, (i / side) as i64 * spacing))
        .collect();
    (points, spacing, side)
}

/// `n` exactly cocircular rational points in counter-clockwise order, from
/// `((1 - t^2) / (1 + t^2), 2t / (1 + t^2))` with `t = m / 2^16`.
fn circle(n: usize) -> Vec<Xy> {
    let d: i64 = 1 << 16;
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(n);
    let radius = int(FRAME / 2);
    let centre = int(FRAME / 2);
    for i in 0..n {
        let theta = -PI + TAU * (i as f64 + 0.5) / n as f64;
        let m = (d as f64 * (theta / 2.0).tan()).round() as i64;
        if !seen.insert(m) {
            continue;
        }
        let den = int(d * d + m * m);
        let x = &centre + &(&(&radius * &int(d * d - m * m)) / &den);
        let y = &centre + &(&(&radius * &int(2 * d * m)) / &den);
        out.push((x, y));
    }
    out
}

fn points_of(kind: &str, n: usize, rng: &mut Rng) -> Vec<Xy> {
    match kind {
        "uniform" => uniform_ints(n, rng).into_iter().map(xy).collect(),
        "clustered" => clustered_ints(n, rng).into_iter().map(xy).collect(),
        "lattice" => lattice_ints(n).0.into_iter().map(xy).collect(),
        "circle" => circle(n),
        other => unreachable!("unknown point kind {other}"),
    }
}

/// Integer points for kinds that have them, for the `f64` entry points; the
/// circle kind becomes nearly (not exactly) cocircular after rounding.
fn f64_points_of(kind: &str, n: usize, rng: &mut Rng) -> Vec<[f64; 2]> {
    match kind {
        "uniform" => to_f64(&uniform_ints(n, rng)),
        "clustered" => to_f64(&clustered_ints(n, rng)),
        "lattice" => to_f64(&lattice_ints(n).0),
        "near-circle" => (0..n)
            .map(|i| {
                let theta = TAU * i as f64 / n as f64;
                [theta.cos(), theta.sin()]
            })
            .collect(),
        other => unreachable!("unknown f64 kind {other}"),
    }
}

/// Add an independent jitter of `j * 2^-k`, `|j| <= 8`, to every coordinate.
fn perturb(points: &[Xy], k: u32, rng: &mut Rng) -> Vec<Xy> {
    let epsilon = &int(1) / &pow2(k);
    points
        .iter()
        .map(|(x, y)| {
            (
                x + &(&epsilon * &int(rng.range(-8, 8))),
                y + &(&epsilon * &int(rng.range(-8, 8))),
            )
        })
        .collect()
}

/// `n - 2` points on the diagonal `y = x` plus two anchors off it.
fn collinear(n: usize) -> Vec<Xy> {
    let count = n.saturating_sub(2).max(1) as i64;
    let spacing = FRAME / count;
    let mut out: Vec<Xy> = (0..count).map(|i| xy((i * spacing, i * spacing))).collect();
    out.push(xy((0, FRAME)));
    out.push(xy((FRAME, 0)));
    out
}

fn degenerate_of(kind: &str, n: usize, k: u32, rng: &mut Rng) -> Vec<Point2> {
    let base = match kind {
        "circle + 2^-k" => circle(n),
        "lattice + 2^-k" => points_of("lattice", n, rng),
        "collinear + 2^-k" => collinear(n),
        other => unreachable!("unknown degenerate kind {other}"),
    };
    lift(&perturb(&base, k, rng))
}

/// A Real equal to zero that no symbolic or certified path can prove zero:
/// `sin(e + 1) - sin(e) cos(1) - cos(e) sin(1)`. Deciding its sign needs the
/// approximate 512-bit terminal.
fn hidden_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

/// Points in one coordinate representation.
fn represented(kind: &str, n: usize, rng: &mut Rng) -> Vec<Point2> {
    match kind {
        "integer" => lift(&points_of("uniform", n, rng)),
        "f64 dyadic" => (0..n)
            .map(|_| {
                Point2::new(
                    Real::try_from(rng.unit()).unwrap(),
                    Real::try_from(rng.unit()).unwrap(),
                )
            })
            .collect(),
        "rational" => {
            let coordinate = |rng: &mut Rng| {
                let q = rng.range(1 << 15, 1 << 16);
                Real::new(Rational::fraction(rng.range(0, q * FRAME), q as u64).unwrap())
            };
            (0..n)
                .map(|_| Point2::new(coordinate(rng), coordinate(rng)))
                .collect()
        }
        "irrational (+pi)" => {
            let pi = Real::pi();
            lift(&points_of("uniform", n, rng))
                .into_iter()
                .map(|p| Point2::new(&p.x + &pi, &p.y + &pi))
                .collect()
        }
        other => unreachable!("unknown representation {other}"),
    }
}

/// Distinct points whose coordinates have `bits` bits in one representation.
fn with_bits(kind: &str, n: usize, bits: u32, rng: &mut Rng) -> Vec<Point2> {
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(n);
    let scale = pow2(bits);
    let mut attempts = 0;
    while out.len() < n && attempts < n * 64 {
        attempts += 1;
        let (x, y) = (big(bits, rng), big(bits, rng));
        let (x, y) = match kind {
            "integer" => (x, y),
            "dyadic" => (&x / &scale, &y / &scale),
            "rational" => (&x / &big(bits, rng), &y / &big(bits, rng)),
            other => unreachable!("unknown bit kind {other}"),
        };
        if !seen.insert((x.to_string(), y.to_string())) {
            continue;
        }
        out.push(Point2::new(Real::new(x), Real::new(y)));
    }
    out
}

// ---------------------------------------------------------------- polygons

/// Counter-clockwise simple polygon of `n` vertices.
fn polygon(kind: &str, n: usize, rng: &mut Rng) -> Vec<Xy> {
    if kind == "convex (cocircular)" {
        circle(n.max(4))
    } else {
        polygon_ints(kind, n, rng).into_iter().map(xy).collect()
    }
}

/// Counter-clockwise integer polygon of `n` vertices for the integer kinds.
fn polygon_ints(kind: &str, n: usize, rng: &mut Rng) -> Vec<(i64, i64)> {
    let n = n.max(4);
    let c = FRAME / 2;
    match kind {
        "star" => (0..n)
            .map(|i| {
                let theta = TAU * i as f64 / n as f64;
                let r = rng.range(FRAME / 8, FRAME / 2) as f64;
                (
                    c + (r * theta.cos()).round() as i64,
                    c + (r * theta.sin()).round() as i64,
                )
            })
            .collect(),
        "comb" => {
            let teeth = n - 2;
            let w = (FRAME / teeth as i64).max(1);
            let mut out: Vec<(i64, i64)> = (0..teeth as i64)
                .map(|i| (i * w, if i % 2 == 0 { 0 } else { FRAME / 4 }))
                .collect();
            out.push(((teeth as i64 - 1) * w, FRAME / 2));
            out.push((0, FRAME / 2));
            out
        }
        "collinear square" => {
            let per_side = (n / 4).max(1) as i64;
            let step = FRAME / per_side;
            let mut out = Vec::with_capacity(n);
            out.extend((0..per_side).map(|i| (i * step, 0)));
            out.extend((0..per_side).map(|i| (FRAME, i * step)));
            out.extend((0..per_side).map(|i| (FRAME - i * step, FRAME)));
            out.extend((0..per_side).map(|i| (0, FRAME - i * step)));
            out
        }
        other => unreachable!("unknown polygon kind {other}"),
    }
}

/// An exterior ring followed by `holes` clockwise square holes laid out on a
/// grid inside `[lo, hi]^2`, with earcut-style hole start indices.
fn with_holes(mut exterior: Vec<Xy>, holes: usize, lo: i64, hi: i64) -> (Vec<Xy>, Vec<usize>) {
    let g = (holes as f64).sqrt().ceil().max(1.0) as i64;
    let cell = (hi - lo) / g;
    let (a, b) = (cell / 4, cell * 3 / 4);
    let mut starts = Vec::with_capacity(holes);
    for j in 0..holes as i64 {
        let (x0, y0) = (lo + (j % g) * cell, lo + (j / g) * cell);
        starts.push(exterior.len());
        exterior.extend(
            [
                (x0 + a, y0 + a),
                (x0 + a, y0 + b),
                (x0 + b, y0 + b),
                (x0 + b, y0 + a),
            ]
            .map(xy),
        );
    }
    (exterior, starts)
}

fn holed(kind: &str, vertices: usize, holes: usize, rng: &mut Rng) -> (Vec<Point2>, Vec<usize>) {
    let (points, starts) = match kind {
        "square exterior" => with_holes(
            [(0, 0), (FRAME, 0), (FRAME, FRAME), (0, FRAME)]
                .map(xy)
                .to_vec(),
            holes,
            FRAME / 8,
            FRAME * 7 / 8,
        ),
        "star exterior" => with_holes(
            polygon("star", vertices, rng),
            holes,
            FRAME / 2 - FRAME / 12,
            FRAME / 2 + FRAME / 12,
        ),
        other => unreachable!("unknown holed kind {other}"),
    };
    (lift(&points), starts)
}

// ------------------------------------------------------------- constraints

struct Pslg {
    ints: Vec<(i64, i64)>,
    points: Vec<Point2>,
    constraints: Vec<Constraint>,
}

impl Pslg {
    fn new(ints: Vec<(i64, i64)>, constraints: Vec<Constraint>) -> Self {
        let points = lift(&ints.iter().copied().map(xy).collect::<Vec<_>>());
        Self {
            ints,
            points,
            constraints,
        }
    }
}

/// Points and up to `c` constraints of one kind. `n` is the point count for
/// every kind except `nested squares`, whose points are the ring corners.
fn pslg(kind: &str, n: usize, c: usize, rng: &mut Rng) -> Pslg {
    match kind {
        "chain" => {
            let ints = uniform_ints(n, rng);
            let mut order: Vec<usize> = (0..ints.len()).collect();
            order.sort_by_key(|&i| ints[i]);
            let constraints = order
                .windows(2)
                .take(c)
                .map(|w| Constraint::new(w[0], w[1]))
                .collect();
            Pslg::new(ints, constraints)
        }
        "crossing segments" => {
            let ints = uniform_ints(n, rng);
            let mut seen = HashSet::new();
            let mut constraints = Vec::with_capacity(c);
            let mut attempts = 0;
            while constraints.len() < c && attempts < c * 16 + 64 {
                attempts += 1;
                let (a, b) = (
                    rng.below(ints.len() as u64) as usize,
                    rng.below(ints.len() as u64) as usize,
                );
                if a != b && seen.insert((a.min(b), a.max(b))) {
                    constraints.push(Constraint::new(a, b));
                }
            }
            Pslg::new(ints, constraints)
        }
        "nested squares" => {
            let rings = (c / 4).max(1) as i64;
            let mut points = Vec::new();
            let mut constraints = Vec::new();
            for i in 0..rings {
                // Concentric squares: corners are collinear along both
                // diagonals, and every ring is cocircular.
                let half = FRAME / 2 * (i + 1) / (rings + 1);
                let c0 = FRAME / 2;
                let start = points.len();
                points.extend([
                    (c0 - half, c0 - half),
                    (c0 + half, c0 - half),
                    (c0 + half, c0 + half),
                    (c0 - half, c0 + half),
                ]);
                for k in 0..4 {
                    constraints.push(Constraint::new(start + k, start + (k + 1) % 4));
                }
            }
            Pslg::new(points, constraints)
        }
        "lattice rows" => {
            let (ints, _, side) = lattice_ints(n);
            let constraints = (0..ints.len().saturating_sub(1))
                .filter(|&i| (i + 1) % side != 0)
                .take(c)
                .map(|i| Constraint::new(i, i + 1))
                .collect();
            Pslg::new(ints, constraints)
        }
        other => unreachable!("unknown constraint kind {other}"),
    }
}

/// Constraint count tied to the point count for sweeps over points.
fn tied_constraints(kind: &str, n: usize) -> usize {
    match kind {
        "crossing segments" => (n as f64).sqrt() as usize,
        "nested squares" => n,
        _ => n / 2,
    }
}

/// Collinear facts for every row and column of a lattice.
fn lattice_facts(n: usize) -> CollinearPointFacts {
    let side = (n as f64).sqrt().ceil().max(1.0) as usize;
    let mut facts = CollinearPointFacts::new(TriangulationCertainty::Certified);
    for row in 0..side {
        let line: Vec<usize> = (row * side..((row + 1) * side).min(n)).collect();
        facts.push_line(&line).unwrap();
    }
    for column in 0..side {
        let line: Vec<usize> = (column..n).step_by(side).collect();
        facts.push_line(&line).unwrap();
    }
    facts
}

// -------------------------------------------------------------- nd / TDS

fn point_d(p: &Point2) -> PointD {
    PointD::new(vec![p.x.clone(), p.y.clone()])
}

fn uniform_d(dimension: usize, n: usize, rng: &mut Rng) -> Vec<PointD> {
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(n);
    while out.len() < n {
        let coordinates: Vec<i64> = (0..dimension).map(|_| rng.range(0, 1 << 16)).collect();
        if seen.insert(coordinates.clone()) {
            out.push(PointD::new(
                coordinates.into_iter().map(Real::from).collect(),
            ));
        }
    }
    out
}

fn nd_points(kind: &str, n: usize, rng: &mut Rng) -> Vec<PointD> {
    match kind {
        "2D uniform" => uniform_d(2, n, rng),
        "3D uniform" => uniform_d(3, n, rng),
        "4D uniform" => uniform_d(4, n, rng),
        "2D circle" => lift(&circle(n)).iter().map(point_d).collect(),
        other => unreachable!("unknown nd kind {other}"),
    }
}

fn delaunay_2d(points: &[Point2]) -> DelaunayTriangulation {
    cdt::delaunay(&APPROX, points)
        .expect("delaunay input")
        .value
}

/// A 2D Delaunay complex from the fast incremental triangulator.
fn complex_2d(points: &[Point2]) -> DelaunayComplex {
    let triangulation = delaunay_2d(points);
    DelaunayComplex::from_parts(
        2,
        points.iter().map(point_d).collect(),
        triangulation
            .triangles()
            .iter()
            .map(|t| Simplex::new(t.to_vec()))
            .collect(),
    )
}

/// A flip of the first interior edge found, preferring an edge whose
/// endpoints differ by exactly `diagonal` in both coordinates (a lattice
/// cell diagonal, whose flip keeps the complex Delaunay).
fn interior_flip(
    triangles: &[[usize; 3]],
    ints: Option<&[(i64, i64)]>,
    diagonal: i64,
) -> BistellarFlipD {
    let mut edges: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for t in triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push(t[(k + 2) % 3]);
        }
    }
    let mut keys: Vec<_> = edges.keys().copied().collect();
    keys.sort_unstable();
    let pick = keys
        .iter()
        .copied()
        .filter(|key| edges[key].len() == 2)
        .find(|&(a, b)| {
            ints.is_none_or(|p| {
                (p[a].0 - p[b].0).abs() == diagonal && (p[a].1 - p[b].1).abs() == diagonal
            })
        })
        .expect("an interior edge");
    let apexes = &edges[&pick];
    BistellarFlipD::new(vec![pick.0, pick.1, apexes[0], apexes[1]], apexes.clone())
}

/// A 2D TDS with reciprocal neighbours from a Delaunay triangulation.
fn tds_2d(points: &[Point2]) -> TriangulationDataStructureD {
    let triangulation = delaunay_2d(points);
    let mut tds = TriangulationDataStructureD::new(2).unwrap();
    let handles: Vec<_> = points
        .iter()
        .map(|p| tds.add_finite_vertex(point_d(p)).unwrap())
        .collect();
    let triangles = triangulation.triangles();
    let mut by_edge: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (index, t) in triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[(k + 1) % 3], t[(k + 2) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(index);
        }
    }
    for (index, t) in triangles.iter().enumerate() {
        let neighbors = (0..3)
            .map(|k| {
                let (a, b) = (t[(k + 1) % 3], t[(k + 2) % 3]);
                by_edge[&(a.min(b), a.max(b))]
                    .iter()
                    .copied()
                    .find(|&other| other != index)
                    .map(CellHandle::new)
            })
            .collect();
        tds.add_cell(Cell::new(
            t.iter().map(|&v| handles[v]).collect(),
            neighbors,
        ))
        .unwrap();
    }
    tds
}

/// Largest power-of-two size at which an input that needs a full Delaunay
/// triangulation (or more) before the timer is still built. Construction time
/// is not bounded by the per-call ceiling, so these sweeps stop here; exactly
/// cocircular inputs are far slower to triangulate and stop earlier.
fn build_cap(atlas: &Atlas, kind: &str) -> i32 {
    let base = if atlas.config().quick { 10 } else { 11 };
    if kind.contains("circle") {
        base - 2
    } else {
        base
    }
}

fn options(
    algorithm: PolygonTriangulationAlgorithm,
    quality: QualityPolicy,
) -> TriangulationOptions {
    TriangulationOptions { algorithm, quality }
}

fn main() {
    let mut atlas = Atlas::from_args("hypertri");
    polygons(&mut atlas);
    delaunay(&mut atlas);
    constrained(&mut atlas);
    dimensional(&mut atlas);
    atlas.finish();
}

fn polygons(atlas: &mut Atlas) {
    let vertices = powers_of_two(2, 14);
    let ring_vertices = powers_of_two(2, 12);

    atlas.operation(
        "polygon",
        "earcut",
        "polygon vertices",
        "Exact ear clipping of one simple ring: cocircular convex, random star, sawtooth comb (reflex teeth, collinear tips), and a square with collinear vertices on every side.",
    );
    for kind in POLYGON_KINDS {
        atlas.sweep(
            kind,
            &vertices,
            |m, rng| lift(&polygon(kind, m as usize, rng)),
            |p| hypertri::earcut(&APPROX, p, &[]).map(|o| o.value.len()),
        );
    }
    atlas.fuzz(
        "random star",
        &powers_of_two(3, 11),
        |m, rng| {
            let n = rng.range(m as i64 / 2, m as i64) as usize;
            lift(&polygon("star", n, rng))
        },
        |p| hypertri::earcut(&APPROX, p, &[]).map(|o| o.value.len()),
    );
    atlas.grid(
        "star exterior + square holes",
        ("holes", &powers_of_two(0, 8)),
        ("exterior vertices", &powers_of_two(3, 12)),
        |h, v, rng| holed("star exterior", v as usize, h as usize, rng),
        |(p, holes)| hypertri::earcut(&APPROX, p, holes).map(|o| o.value.len()),
    );

    atlas.operation(
        "polygon",
        "earcut_report",
        "polygon vertices",
        "Ear clipping with hot-loop diagnostics (ear tests, containment candidates, split fallbacks).",
    );
    for kind in ["star", "comb"] {
        atlas.sweep(
            kind,
            &vertices,
            |m, rng| lift(&polygon(kind, m as usize, rng)),
            |p| {
                hypertri::earcut_report(&APPROX, p, &[])
                    .map(|o| (o.value.triangles.len(), o.value.diagnostics.ear_tests))
            },
        );
    }

    atlas.operation(
        "polygon",
        "earcut with holes",
        "holes",
        "Ear clipping of a polygon with square holes (4 vertices each) bridged into the exterior; hole count grows on a fixed square or 256-vertex star exterior.",
    );
    for kind in ["square exterior", "star exterior"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |h, rng| holed(kind, 256, h as usize, rng),
            |(p, holes)| hypertri::earcut(&APPROX, p, holes).map(|o| o.value.len()),
        );
    }

    atlas.operation(
        "polygon",
        "rings_from_hole_indices",
        "holes",
        "Validate earcut-style hole indices into ring ranges (structural, no predicates).",
    );
    atlas.sweep(
        "square exterior",
        &powers_of_two(0, 14),
        |h, rng| holed("square exterior", 4, h as usize, rng),
        |(p, holes)| hypertri::polygon::rings_from_hole_indices(p, holes).map(|r| r.holes().len()),
    );

    atlas.operation(
        "polygon",
        "triangulate_simple_ring",
        "polygon vertices",
        "Validated simple-ring triangulation: quadratic edge-pair simplicity check, then ear clipping and orientation.",
    );
    for kind in POLYGON_KINDS {
        atlas.sweep(
            kind,
            &ring_vertices,
            |m, rng| lift(&polygon(kind, m as usize, rng)),
            |p| hypertri::triangulate_simple_ring(&APPROX, p).map(|o| o.value.triangles.len()),
        );
    }

    atlas.operation(
        "polygon",
        "triangulate_simple_ring_around",
        "polygon vertices",
        "Simple-ring triangulation that fans from vertex 0 wherever it is visible (cubic clipping at worst).",
    );
    for kind in POLYGON_KINDS {
        atlas.sweep(
            kind,
            &ring_vertices,
            |m, rng| lift(&polygon(kind, m as usize, rng)),
            |p| {
                hypertri::triangulate_simple_ring_around(&APPROX, p, 0)
                    .map(|o| o.value.triangles.len())
            },
        );
    }

    atlas.operation(
        "polygon",
        "f64::earcut",
        "polygon vertices",
        "Ear clipping of finite f64 input lifted exactly to dyadic reals (lifting is inside the timer).",
    );
    for kind in ["star", "comb", "near-circle"] {
        atlas.sweep(
            kind,
            &vertices,
            |m, rng| {
                let n = m as usize;
                if kind == "near-circle" {
                    f64_points_of("near-circle", n.max(4), rng)
                } else {
                    to_f64(&polygon_ints(kind, n, rng))
                }
            },
            |p: &Vec<[f64; 2]>| hypertri::f64::earcut(&APPROX, p, &[]).map(|o| o.value.len()),
        );
    }

    let algorithms = [
        (
            "auto, preserve boundary",
            options(
                PolygonTriangulationAlgorithm::Auto,
                QualityPolicy::PreserveBoundary,
            ),
        ),
        (
            "auto, prefer Delaunay",
            options(
                PolygonTriangulationAlgorithm::Auto,
                QualityPolicy::PreferDelaunay,
            ),
        ),
        (
            "earcut",
            options(
                PolygonTriangulationAlgorithm::Earcut,
                QualityPolicy::PreserveBoundary,
            ),
        ),
        (
            "constrained Delaunay",
            options(
                PolygonTriangulationAlgorithm::ConstrainedDelaunay,
                QualityPolicy::PreferDelaunay,
            ),
        ),
    ];
    atlas.operation(
        "polygon",
        "triangulate_polygon (runtime select)",
        "polygon vertices",
        "Runtime algorithm selection on a random star polygon: each kind is one TriangulationOptions setting.",
    );
    for (kind, option) in algorithms {
        atlas.sweep(
            kind,
            &ring_vertices,
            |m, rng| PolygonInput::new(lift(&polygon("star", m as usize, rng)), Vec::new()),
            |input| hypertri::triangulate_polygon(&APPROX, input, option).map(|o| o.value.len()),
        );
    }

    atlas.operation(
        "polygon",
        "triangulate_polygon with holes",
        "holes",
        "Runtime polygon triangulation of a square exterior with square holes, earcut versus the constrained-Delaunay path.",
    );
    for (kind, option) in [algorithms[2], algorithms[3]] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |h, rng| {
                let (p, holes) = holed("square exterior", 4, h as usize, rng);
                PolygonInput::new(p, holes)
            },
            |input| hypertri::triangulate_polygon(&APPROX, input, option).map(|o| o.value.len()),
        );
    }

    atlas.operation(
        "polygon",
        "triangulate_polygon_with_report",
        "polygon vertices",
        "Runtime selection plus the selection report (clones the polygon facts).",
    );
    for kind in ["star", "comb"] {
        atlas.sweep(
            kind,
            &ring_vertices,
            |m, rng| PolygonInput::new(lift(&polygon(kind, m as usize, rng)), Vec::new()),
            |input| {
                hypertri::triangulate_polygon_with_report(
                    &APPROX,
                    input,
                    TriangulationOptions::default(),
                )
                .map(|o| o.value.0.len())
            },
        );
    }

    atlas.operation(
        "polygon",
        "PolygonInput::new (facts)",
        "polygon vertices",
        "Build a polygon input and its structural facts (exact-rational, dyadic, shared denominators, axis-aligned edges) in one coordinate representation; the vertex buffer is cloned inside the timer.",
    );
    for kind in ["integer", "f64 dyadic", "rational", "irrational (+pi)"] {
        atlas.sweep(
            kind,
            &vertices,
            |m, rng| represented(kind, m as usize, rng),
            |p| {
                let input = PolygonInput::new(p.clone(), Vec::new());
                input.facts().known_degenerate_edge_count() + input.facts().vertex_count
            },
        );
    }
}

fn delaunay(atlas: &mut Atlas) {
    let counts = powers_of_two(3, 14);
    let perturbations: Vec<f64> = [
        1, 4, 8, 16, 24, 32, 40, 48, 52, 56, 60, 64, 80, 96, 128, 192, 256, 384, 512,
    ]
    .map(f64::from)
    .to_vec();

    for (name, run) in [
        (
            "delaunay",
            cdt::delaunay as fn(&TriangulationContext, &[Point2]) -> _,
        ),
        ("delaunay_spatial", cdt::delaunay_spatial),
    ] {
        atlas.operation(
            "delaunay",
            name,
            "points",
            &format!(
                "Exact 2D Delaunay triangulation ({}) of uniform, clustered, lattice (cocircular cells, collinear rows) and exactly cocircular rational points.",
                if name == "delaunay" {
                    "incremental insertion in caller order"
                } else {
                    "BRIO-style spatial insertion order"
                }
            ),
        );
        for kind in POINT_KINDS {
            atlas.sweep(
                kind,
                &counts,
                |m, rng| lift(&points_of(kind, m as usize, rng)),
                |p| run(&APPROX, p).map(|o| o.value.triangles().len()),
            );
        }
        atlas.fuzz(
            "mixed kinds, jittered",
            &powers_of_two(3, 10),
            |m, rng| {
                let kind = POINT_KINDS[rng.below(4) as usize];
                let n = rng.range(m as i64 / 2, m as i64) as usize;
                let base = points_of(kind, n, rng);
                let k = rng.range(1, 128) as u32;
                lift(&if rng.coin() {
                    perturb(&base, k, rng)
                } else {
                    base
                })
            },
            |p| run(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }

    atlas.operation(
        "delaunay",
        "delaunay by representation",
        "points",
        "Delaunay triangulation of uniform points in one coordinate representation: integers, f64 values lifted to dyadics, rationals with random 16-bit denominators, and integers translated by pi.",
    );
    for kind in ["integer", "f64 dyadic", "rational", "irrational (+pi)"] {
        atlas.sweep(
            kind,
            &counts,
            |m, rng| represented(kind, m as usize, rng),
            |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }

    atlas.operation(
        "delaunay",
        "delaunay by coordinate bits",
        "coordinate bits",
        "Delaunay triangulation of 128 random points whose coordinates are b-bit integers, b-bit dyadics (x / 2^b) or b-bit / b-bit rationals; shows the predicate cost of big coordinates.",
    );
    let bits = powers_of_two(3, 12);
    for kind in ["integer", "dyadic", "rational"] {
        atlas.sweep(
            kind,
            &bits,
            |b, rng| with_bits(kind, 128, b as u32, rng),
            |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }
    atlas.grid(
        "integer",
        ("points", &powers_of_two(3, 11)),
        ("coordinate bits", &powers_of_two(3, 11)),
        |n, b, rng| with_bits("integer", n as usize, b as u32, rng),
        |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
    );

    atlas.operation(
        "delaunay",
        "delaunay near degeneracy",
        "perturbation exponent k (jitter j * 2^-k, |j| <= 8)",
        "Delaunay triangulation of 256 points that are exactly degenerate (cocircular rational circle, lattice, or collinear diagonal plus two anchors) after a jitter of size 2^-k: as k grows, cheap filters stop deciding and predicates escalate to exact big-rational arithmetic.",
    );
    for kind in ["circle + 2^-k", "lattice + 2^-k", "collinear + 2^-k"] {
        atlas.sweep(
            kind,
            &perturbations,
            |k, rng| degenerate_of(kind, 256, k as u32, rng),
            |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }
    atlas.fuzz(
        "lattice + 2^-k, random k",
        &perturbations,
        |k, rng| {
            let k = rng.range((k as i64 / 2).max(1), k as i64) as u32;
            degenerate_of("lattice + 2^-k", 128, k, rng)
        },
        |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
    );
    let grid_perturbations = [1.0, 8.0, 24.0, 48.0, 64.0, 128.0, 256.0, 512.0];
    for kind in ["circle + 2^-k", "lattice + 2^-k"] {
        atlas.grid(
            kind,
            ("points", &powers_of_two(3, 10)),
            ("perturbation exponent k", &grid_perturbations),
            |n, k, rng| degenerate_of(kind, n as usize, k as u32, rng),
            |p| cdt::delaunay(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }

    atlas.operation(
        "delaunay",
        "delaunay by predicate policy",
        "points",
        "Delaunay triangulation under STRICT and APPROXIMATE_512 policies, for a rational lattice and for a lattice whose every 8th point is shifted by a zero no certified path can prove (sin(e+1) - sin e cos 1 - cos e sin 1). STRICT refuses that input (error points are expected); APPROXIMATE_512 decides it with the 512-bit terminal.",
    );
    for (kind, context, hidden) in [
        ("STRICT, rational lattice", STRICT, false),
        ("APPROXIMATE_512, rational lattice", APPROX, false),
        ("STRICT, lattice + hidden zero", STRICT, true),
        ("APPROXIMATE_512, lattice + hidden zero", APPROX, true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(3, 12),
            |m, rng| {
                let mut points = lift(&points_of("lattice", m as usize, rng));
                if hidden {
                    let zero = hidden_zero();
                    for p in points.iter_mut().step_by(8) {
                        p.x = &p.x + &zero;
                    }
                }
                points
            },
            |p| cdt::delaunay(&context, p).map(|o| o.value.triangles().len()),
        );
    }

    let f64_kinds = ["uniform", "clustered", "lattice", "near-circle"];
    for (name, run) in [
        (
            "f64::delaunay",
            hypertri::f64::delaunay as fn(&TriangulationContext, &[[f64; 2]]) -> _,
        ),
        ("f64::delaunay_spatial", hypertri::f64::delaunay_spatial),
    ] {
        atlas.operation(
            "delaunay",
            name,
            "points",
            "Delaunay triangulation of finite f64 points lifted exactly to dyadics (lifting is inside the timer). Near-circle points (cos, sin) are rounded, so nearly but not exactly cocircular.",
        );
        for kind in f64_kinds {
            atlas.sweep(
                kind,
                &counts,
                |m, rng| f64_points_of(kind, m as usize, rng),
                |p| run(&APPROX, p).map(|o| o.value.triangles().len()),
            );
        }
        atlas.fuzz(
            "uniform, random exponent span",
            &powers_of_two(3, 10),
            |m, rng| {
                // Coordinates spanning 2^-e .. 2^e stress the dyadic lifts.
                let e = rng.range(0, 60) as i32;
                (0..m as usize)
                    .map(|_| {
                        [
                            (rng.unit() - 0.5) * 2f64.powi(rng.range(-e as i64, e as i64) as i32),
                            (rng.unit() - 0.5) * 2f64.powi(rng.range(-e as i64, e as i64) as i32),
                        ]
                    })
                    .collect::<Vec<_>>()
            },
            |p| run(&APPROX, p).map(|o| o.value.triangles().len()),
        );
    }

    atlas.operation(
        "delaunay",
        "DelaunayTriangulation::validate",
        "points",
        "Re-validate a completed Delaunay triangulation (orientation and empty-circle of every edge); the triangulation is built outside the timer.",
    );
    for kind in ["uniform", "lattice", "circle"] {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |m, rng| delaunay_2d(&lift(&points_of(kind, m as usize, rng))),
            |t| t.validate(&APPROX).map(|_| t.triangles().len()),
        );
    }
}

fn constrained(atlas: &mut Atlas) {
    let constraint_counts = powers_of_two(0, 10);
    let point_counts = powers_of_two(3, 12);
    let kinds = [
        "chain",
        "crossing segments",
        "nested squares",
        "lattice rows",
    ];
    let planar = ["chain", "nested squares", "lattice rows"];

    atlas.operation(
        "constrained",
        "constrained_delaunay by constraints",
        "constraints",
        "Constrained Delaunay triangulation of 1024 points with c constraints: an x-monotone chain, random crossing segments (exact intersection vertices are inserted), nested closed squares (routed through the polygon-with-holes path; its points are just the c ring corners), and collinear lattice-row edges.",
    );
    for kind in kinds {
        // Random crossing segments grow intersection vertices quadratically;
        // past 128 segments a single call takes tens of seconds.
        let counts = if kind == "crossing segments" {
            powers_of_two(0, 7)
        } else {
            constraint_counts.clone()
        };
        atlas.sweep(
            kind,
            &counts,
            |c, rng| pslg(kind, 1024, c as usize, rng),
            |g| {
                cdt::constrained_delaunay(&APPROX, &g.points, &g.constraints)
                    .map(|o| o.value.triangles().len())
            },
        );
    }
    atlas.fuzz(
        "crossing segments",
        &powers_of_two(0, 7),
        |c, rng| {
            let n = rng.range(16, 512) as usize;
            pslg("crossing segments", n, c as usize, rng)
        },
        |g| {
            cdt::constrained_delaunay(&APPROX, &g.points, &g.constraints)
                .map(|o| o.value.triangles().len())
        },
    );
    atlas.grid(
        "crossing segments",
        ("constraints", &powers_of_two(0, 8)),
        ("points", &powers_of_two(4, 11)),
        |c, n, rng| pslg("crossing segments", n as usize, c as usize, rng),
        |g| {
            cdt::constrained_delaunay(&APPROX, &g.points, &g.constraints)
                .map(|o| o.value.triangles().len())
        },
    );

    atlas.operation(
        "constrained",
        "constrained_delaunay by points",
        "points",
        "Constrained Delaunay triangulation with the constraint count tied to the point count: chain (n/2 edges), crossing segments (sqrt n), nested squares (n corners), lattice rows (n/2 edges).",
    );
    for kind in kinds {
        atlas.sweep(
            kind,
            &point_counts,
            |n, rng| pslg(kind, n as usize, tied_constraints(kind, n as usize), rng),
            |g| {
                cdt::constrained_delaunay(&APPROX, &g.points, &g.constraints)
                    .map(|o| o.value.triangles().len())
            },
        );
    }

    atlas.operation(
        "constrained",
        "constrained_delaunay_convex_hull",
        "points",
        "Constrained Delaunay over the full convex hull of an already planar PSLG (chain and lattice rows with n/2 edges, nested squares with n corners): closed rings separate cells instead of cutting holes.",
    );
    for kind in planar {
        atlas.sweep(
            kind,
            &point_counts,
            |n, rng| pslg(kind, n as usize, tied_constraints(kind, n as usize), rng),
            |g| {
                cdt::constrained_delaunay_convex_hull(&APPROX, &g.points, &g.constraints)
                    .map(|o| o.value.triangles().len())
            },
        );
    }

    atlas.operation(
        "constrained",
        "constrained_triangulation_convex_hull",
        "points",
        "Topology-only constrained triangulation of a planar PSLG (greedy endpoint-pair diagonals, no Delaunay legalization; chain and lattice rows with n/2 edges, nested squares with n corners).",
    );
    for kind in planar {
        atlas.sweep(
            kind,
            &point_counts,
            |n, rng| pslg(kind, n as usize, tied_constraints(kind, n as usize), rng),
            |g| {
                cdt::constrained_triangulation_convex_hull(&APPROX, &g.points, &g.constraints)
                    .map(|o| o.value.len())
            },
        );
    }

    atlas.operation(
        "constrained",
        "convex hull triangulation with collinear facts",
        "points",
        "Topology-only triangulation of a lattice with its row edges protected, with and without caller-proved collinear facts for every row and column, for a rational lattice and for one translated by pi (where proving orientation zero is costly).",
    );
    for (kind, irrational, facts) in [
        ("rational, no facts", false, false),
        ("rational, row/column facts", false, true),
        ("+pi, no facts", true, false),
        ("+pi, row/column facts", true, true),
    ] {
        atlas.sweep(
            kind,
            &point_counts,
            |n, rng| {
                let n = n as usize;
                let mut g = pslg("lattice rows", n, n, rng);
                if irrational {
                    let pi = Real::pi();
                    for p in &mut g.points {
                        *p = Point2::new(&p.x + &pi, &p.y + &pi);
                    }
                }
                let facts = if facts {
                    lattice_facts(n)
                } else {
                    CollinearPointFacts::new(TriangulationCertainty::Certified)
                };
                (g, facts)
            },
            |(g, facts)| {
                cdt::constrained_triangulation_convex_hull_with_collinear_facts(
                    &APPROX,
                    &g.points,
                    &g.constraints,
                    facts,
                )
                .map(|o| o.value.len())
            },
        );
    }

    atlas.operation(
        "constrained",
        "ConstrainedTriangulation::validate",
        "points",
        "Re-validate a completed constrained Delaunay triangulation (the triangulation is built outside the timer); the 'unconstrained edges' kinds run validate_unconstrained_edges_are_delaunay instead.",
    );
    for (kind, shape, unconstrained) in [
        ("chain", "chain", false),
        ("crossing segments", "crossing segments", false),
        ("chain, unconstrained edges", "chain", true),
        ("crossing, unconstrained edges", "crossing segments", true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |n, rng| {
                let n = n as usize;
                let g = pslg(shape, n, tied_constraints(shape, n), rng);
                cdt::constrained_delaunay(&APPROX, &g.points, &g.constraints)
                    .expect("constrained input")
                    .value
            },
            |t| {
                if unconstrained {
                    t.validate_unconstrained_edges_are_delaunay(&APPROX)
                } else {
                    t.validate(&APPROX)
                }
                .map(|_| t.triangles().len())
            },
        );
    }

    atlas.operation(
        "constrained",
        "f64::constrained_delaunay",
        "points",
        "Constrained Delaunay of f64 points lifted exactly (inside the timer): x-monotone chain (n/2 edges) and crossing segments (sqrt n).",
    );
    for kind in ["chain", "crossing segments"] {
        atlas.sweep(
            kind,
            &point_counts,
            |n, rng| {
                let n = n as usize;
                let g = pslg(kind, n, tied_constraints(kind, n), rng);
                (to_f64(&g.ints), g.constraints)
            },
            |(p, c)| {
                hypertri::f64::constrained_delaunay(&APPROX, p, c)
                    .map(|o| o.value.triangles().len())
            },
        );
    }
}

fn dimensional(atlas: &mut Atlas) {
    let small = ladder(4, 96, 12);

    atlas.operation(
        "nd",
        "nd::delaunay_complex",
        "points",
        "Exhaustive D-dimensional Delaunay oracle: every (d+1)-subset gets an exact orientation and empty-sphere test (O(n^(d+2)) predicates). The 2D circle kind is exactly cospherical, so every triangle passes.",
    );
    for kind in ["2D uniform", "3D uniform", "4D uniform", "2D circle"] {
        atlas.sweep(
            kind,
            &small,
            |n, rng| nd_points(kind, n as usize, rng),
            |p| nd::delaunay_complex(&APPROX, p).map(|o| o.value.cells().len()),
        );
    }
    atlas.fuzz(
        "random dimension 2-4",
        &ladder(4, 24, 6),
        |n, rng| {
            let kind = ["2D uniform", "3D uniform", "4D uniform"][rng.below(3) as usize];
            nd_points(kind, n as usize, rng)
        },
        |p| nd::delaunay_complex(&APPROX, p).map(|o| o.value.cells().len()),
    );

    atlas.operation(
        "nd",
        "DelaunayComplex::validate",
        "points",
        "Validate a Delaunay complex (affine independence and empty sphere for every cell against every point, O(cells * n)). 2D complexes come from the incremental triangulator, 3D from the oracle; both are built outside the timer.",
    );
    for kind in ["2D uniform", "2D lattice", "3D uniform"] {
        let sizes = if kind == "3D uniform" {
            ladder(5, 24, 6)
        } else {
            powers_of_two(3, build_cap(atlas, kind))
        };
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                let n = n as usize;
                match kind {
                    "2D uniform" => complex_2d(&lift(&points_of("uniform", n, rng))),
                    "2D lattice" => complex_2d(&lift(&points_of("lattice", n, rng))),
                    _ => {
                        nd::delaunay_complex(&APPROX, &uniform_d(3, n, rng))
                            .unwrap()
                            .value
                    }
                }
            },
            |c| c.validate(&APPROX).map(|_| c.cells().len()),
        );
    }

    atlas.operation(
        "nd",
        "DelaunayComplex::insert_point_oracle",
        "points in the complex",
        "Insert one point into a Delaunay complex: validation, exact conflict cells, then an exhaustive oracle rebuild.",
    );
    for kind in ["2D uniform", "3D uniform"] {
        atlas.sweep(
            kind,
            &small,
            |n, rng| {
                let dimension = if kind == "2D uniform" { 2 } else { 3 };
                let mut points = uniform_d(dimension, n as usize + 1, rng);
                let extra = points.pop().unwrap();
                let complex = nd::delaunay_complex(&APPROX, &points).unwrap().value;
                (complex, extra)
            },
            |(complex, extra)| {
                complex
                    .insert_point_oracle(&APPROX, extra.clone())
                    .map(|o| o.value.new_cell_count())
            },
        );
    }

    let flips = |kind: &str, n: usize, rng: &mut Rng| {
        let (complex, flip) = if kind == "2D lattice (legal flip)" {
            let (ints, spacing, _) = lattice_ints(n);
            let points = lift(&ints.iter().copied().map(xy).collect::<Vec<_>>());
            let complex = complex_2d(&points);
            let triangles: Vec<[usize; 3]> = complex
                .cells()
                .iter()
                .map(|c| [c.indices()[0], c.indices()[1], c.indices()[2]])
                .collect();
            let flip = interior_flip(&triangles, Some(&ints), spacing);
            (complex, flip)
        } else {
            let points = lift(&points_of("uniform", n, rng));
            let complex = complex_2d(&points);
            let triangles: Vec<[usize; 3]> = complex
                .cells()
                .iter()
                .map(|c| [c.indices()[0], c.indices()[1], c.indices()[2]])
                .collect();
            let flip = interior_flip(&triangles, None, 0);
            (complex, flip)
        };
        (complex, flip)
    };

    atlas.operation(
        "nd",
        "DelaunayComplex::validate_bistellar_flip",
        "points in the complex",
        "Validate a 2-2 flip: cell lookup, inserted-cell orientation and empty-circle tests. On a lattice the flipped cell is cocircular (legal); on uniform points the flip is blocked by Delaunay legality (a successful 'blocked' report).",
    );
    for kind in ["2D lattice (legal flip)", "2D uniform (blocked flip)"] {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |n, rng| flips(kind, n as usize, rng),
            |(complex, flip)| {
                let report = complex.validate_bistellar_flip(&APPROX, flip).value;
                report.reason().is_none()
            },
        );
    }

    atlas.operation(
        "nd",
        "DelaunayComplex::flip_oracle",
        "points in the complex",
        "Apply a legal 2-2 flip on a lattice complex and re-validate the whole result (O(cells * n)).",
    );
    atlas.sweep(
        "2D lattice (legal flip)",
        &powers_of_two(3, build_cap(atlas, "lattice")),
        |n, rng| flips("2D lattice (legal flip)", n as usize, rng),
        |(complex, flip)| {
            complex
                .flip_oracle(&APPROX, flip)
                .map(|o| o.value.result().cells().len())
        },
    );

    let tds_kinds = ["uniform", "lattice", "circle"];
    // The same TDS inputs serve every TDS operation; build each once.
    let mut tds_cache: HashMap<(&str, u64), TriangulationDataStructureD> = HashMap::new();
    let mut tds = |kind: &'static str, n: f64, rng: &mut Rng| {
        tds_cache
            .entry((kind, n as u64))
            .or_insert_with(|| tds_2d(&lift(&points_of(kind, n as usize, rng))))
            .clone()
    };
    atlas.operation(
        "nd",
        "TDS validate_combinatorial_report",
        "points",
        "Combinatorial validation of a 2D triangulation data structure (arity, reciprocal neighbours, facet degrees) built from a Delaunay triangulation outside the timer.",
    );
    for kind in tds_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |n, rng| tds(kind, n, rng),
            |tds| tds.validate_combinatorial_report().is_valid(),
        );
    }
    atlas.operation(
        "nd",
        "TDS validate_manifold_report",
        "points",
        "Finite-facet manifold validation (facet incidences and induced orientations) of a 2D TDS.",
    );
    for kind in tds_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |n, rng| tds(kind, n, rng),
            |tds| {
                tds.validate_manifold_report(TdsBoundaryPolicyD::AllowBoundary)
                    .is_valid()
            },
        );
    }
    atlas.operation(
        "nd",
        "TDS validate_geometric_report",
        "points",
        "Geometric validation of a 2D TDS: exact orientation of every finite cell and cospherical boundary counts.",
    );
    for kind in tds_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(3, build_cap(atlas, kind)),
            |n, rng| tds(kind, n, rng),
            |tds| tds.validate_geometric_report(&APPROX).value.is_valid(),
        );
    }
    atlas.operation(
        "nd",
        "TriangulationD::new",
        "points",
        "Wrap a TDS as a triangulation (combinatorial validation); the TDS clone is inside the timer.",
    );
    atlas.sweep(
        "uniform",
        &powers_of_two(3, build_cap(atlas, "uniform")),
        |n, rng| tds("uniform", n, rng),
        |tds| TriangulationD::new(tds.clone()).map(|t| t.tds().cells().len()),
    );
}
