//! Performance atlas for Hypercurve: exact planar curve construction,
//! evaluation, bounds, curve/curve intersection, parameter location,
//! offsets, region arrangements and Booleans, spline editing, flattening and
//! SVG exchange, across curve families, coordinate representations and sizes,
//! curve degree, near-tangency, intersection multiplicity, offset distance
//! relative to curvature and rational weight magnitude. Run through
//! `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypercurve --all-features --example perf_atlas -- --out atlas.json`.
//!
//! Conventions used by every operation below:
//!
//! - Coordinate size is introduced by an exact similarity applied to a fixed
//!   template shape (rotation by a Pythagorean-triple angle plus a translation
//!   whose parts have the requested bit length), so the geometry, its
//!   tangencies and its curvature are identical at every size. "integer"
//!   uses an integer similarity (distances scale with it), "rational" a unit
//!   rational rotation and rational translation, and "irrational" a rotation
//!   by an angle whose cosine and sine are square roots of `bits`-bit
//!   rationals.
//! - "cold" operations rebuild their curves, paths or regions from exact
//!   rationals inside the timed call, so no cache retained by an earlier call
//!   is reused; the construction cost is included and measured separately
//!   under `construction`. "warm" queries run on a prebuilt object, as
//!   repeated queries would, so retained caches are reused after the first call.
//! - Heavy exact operations run under a cancellable, unlimited work budget
//!   (`evaluate_with_cancellable_budget`) with a wall-clock watchdog at twice
//!   the harness ceiling, so a pathological input ends its series instead of
//!   hanging the atlas. Cancellation is cooperative: uninstrumented exact
//!   arithmetic can still overrun the watchdog (single calls of several
//!   seconds were observed). Incomplete intersection results (typed blockers),
//!   uncertain or failed exact results are reported as errors.
use std::f64::consts::PI;
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use hypercurve::{
    BezierFlatteningOptions, BooleanOp, BulgeVertex2, CancellationToken, CircularArc2, Contour2,
    CubicBezier2, Curve2, CurveGeometry2, CurveParameter2, CurveParameterRange2, CurvePath2,
    CurvePoint2, CurvePointLocations2, CurveRegion2, CurveString2, FillRule,
    FiniteProjectionOptions, LineSeg2, MarchingSquaresGrid, NurbsCurve2, OffsetCap,
    OffsetCornerStyle2, Point2, PolynomialSplineCurve2, PredicatePolicy, QuadraticBezier2,
    Rational, RationalBezier2, RationalQuadraticBezier2, Real, Segment2, WorkBudget,
    evaluate_with_cancellable_budget, marching_squares,
};
use num::{BigInt, BigUint};
use perf_atlas::{Atlas, Rng, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

// ---------------------------------------------------------------------------
// Watchdog and budgeted evaluation.

static WATCH: Mutex<Option<(Instant, CancellationToken)>> = Mutex::new(None);
static LIMIT: OnceLock<Duration> = OnceLock::new();

fn start_watchdog(limit: Duration) {
    let _ = LIMIT.set(limit);
    std::thread::spawn(|| {
        loop {
            std::thread::sleep(Duration::from_millis(2));
            if let Ok(guard) = WATCH.lock()
                && let Some((deadline, token)) = guard.as_ref()
                && Instant::now() > *deadline
            {
                token.cancel();
            }
        }
    });
}

/// Run an exact operation under STRICT with an unlimited work budget that a
/// watchdog cancels after twice the harness ceiling. Resource exhaustion,
/// uncertainty and domain errors all yield `None`.
fn guarded<T, E>(work: impl FnOnce() -> Result<T, E>) -> Option<T> {
    let token = CancellationToken::new();
    let limit = LIMIT.get().copied().unwrap_or(Duration::from_secs(10));
    if let Ok(mut watch) = WATCH.lock() {
        *watch = Some((Instant::now() + limit, token.clone()));
    }
    let outcome = evaluate_with_cancellable_budget(
        PredicatePolicy::STRICT,
        WorkBudget::UNLIMITED,
        &token,
        work,
    );
    if let Ok(mut watch) = WATCH.lock() {
        *watch = None;
    }
    outcome.result.ok()?.certified()?.ok()
}

// ---------------------------------------------------------------------------
// Exact numbers.

fn big(rng: &mut Rng, bits: u32) -> BigInt {
    BigInt::parse_bytes(rng.decimal(bits.max(1)).as_bytes(), 10).expect("decimal digits")
}

fn signed(rng: &mut Rng, value: BigInt) -> BigInt {
    if rng.coin() { -value } else { value }
}

fn int(value: i64) -> Rational {
    Rational::new(value)
}

fn ratio(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).expect("nonzero denominator")
}

fn fraction(numerator: BigInt, denominator: &BigInt) -> Rational {
    Rational::from_bigint_fraction(numerator, denominator.magnitude().clone())
        .expect("nonzero denominator")
}

/// A signed rational whose numerator and denominator both have `bits` bits.
fn random_rational(rng: &mut Rng, bits: u32) -> Rational {
    let numerator = big(rng, bits);
    let numerator = signed(rng, numerator);
    let denominator = big(rng, bits);
    fraction(numerator, &denominator)
}

/// Exact `2^exponent`.
fn pow2(exponent: i32) -> Rational {
    if exponent >= 0 {
        Rational::from_bigint(BigInt::from(1) << exponent as usize)
    } else {
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1u8) << (-exponent) as usize)
            .expect("nonzero denominator")
    }
}

/// A non-dyadic parameter in (0, 1) with about 30 bits.
fn unit_parameter(rng: &mut Rng) -> Rational {
    ratio(1 + rng.below(1_000_000_006) as i64, 1_000_000_007)
}

fn real(value: &Rational) -> Real {
    Real::new(value.clone())
}

fn reals(values: &[Rational]) -> Vec<Real> {
    values.iter().map(real).collect()
}

/// Force a numeric approximation of an exact point, so lazily represented
/// coordinates are actually evaluated.
fn force(point: &CurvePoint2) -> Option<f64> {
    if let Some(p) = point.coordinates() {
        return Some(approximate(p.x())? + approximate(p.y())?);
    }
    let bounds = point.bounds().ok()?;
    Some(approximate(bounds.min_x())? + approximate(bounds.max_y())?)
}

/// f64 approximation, rescaled by 2^-1200 when the value exceeds f64 range
/// (1024-bit integer placements).
fn approximate(value: &Real) -> Option<f64> {
    value
        .to_f64_lossy()
        .or_else(|| (value * &real(&pow2(-1200))).to_f64_lossy())
}

// ---------------------------------------------------------------------------
// Coordinate representations.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Num {
    Integer,
    Rational,
    Irrational,
}

const NUMS: [(&str, Num); 3] = [
    ("integer", Num::Integer),
    ("rational", Num::Rational),
    ("irrational", Num::Irrational),
];

/// An exact similarity `p -> R p + t` placing a template shape. For
/// [`Num::Irrational`], `c` and `s` hold the squares of the rotation entries.
#[derive(Clone, Debug)]
struct Embed {
    num: Num,
    c: Rational,
    s: Rational,
    tx: Rational,
    ty: Rational,
    /// Factor by which the similarity scales lengths.
    scale: Rational,
}

impl Embed {
    fn identity() -> Self {
        Self {
            num: Num::Rational,
            c: int(1),
            s: int(0),
            tx: int(0),
            ty: int(0),
            scale: int(1),
        }
    }

    fn random(num: Num, bits: u32, rng: &mut Rng) -> Self {
        let half = (bits / 2).max(1);
        match num {
            Num::Integer | Num::Rational => {
                let a = big(rng, half);
                let b = big(rng, half);
                let c = &a * &a - &b * &b;
                let s = BigInt::from(2) * &a * &b;
                let norm = &a * &a + &b * &b;
                if num == Num::Integer {
                    let tx = big(rng, bits);
                    let tx = signed(rng, tx);
                    let ty = big(rng, bits);
                    let ty = signed(rng, ty);
                    Self {
                        num,
                        c: Rational::from_bigint(c),
                        s: Rational::from_bigint(s),
                        tx: Rational::from_bigint(tx),
                        ty: Rational::from_bigint(ty),
                        scale: Rational::from_bigint(norm),
                    }
                } else {
                    Self {
                        num,
                        c: fraction(c, &norm),
                        s: fraction(s, &norm),
                        tx: random_rational(rng, bits),
                        ty: random_rational(rng, bits),
                        scale: int(1),
                    }
                }
            }
            Num::Irrational => {
                let p = big(rng, bits);
                let r = big(rng, bits);
                let norm = &p + &r;
                Self {
                    num,
                    c: fraction(p, &norm),
                    s: fraction(r, &norm),
                    tx: random_rational(rng, bits),
                    ty: random_rational(rng, bits),
                    scale: int(1),
                }
            }
        }
    }

    fn rational(bits: u32, rng: &mut Rng) -> Self {
        Self::random(Num::Rational, bits, rng)
    }

    fn frame(&self) -> Frame {
        let (c, s) = if self.num == Num::Irrational {
            (
                real(&self.c).sqrt().expect("positive"),
                real(&self.s).sqrt().expect("positive"),
            )
        } else {
            (real(&self.c), real(&self.s))
        };
        Frame {
            c,
            s,
            tx: real(&self.tx),
            ty: real(&self.ty),
        }
    }

    fn length(&self, length: &Rational) -> Rational {
        length * &self.scale
    }
}

/// Exact reals of an [`Embed`], built inside timed calls so irrational
/// approximation caches start cold.
struct Frame {
    c: Real,
    s: Real,
    tx: Real,
    ty: Real,
}

impl Frame {
    fn point(&self, p: &P) -> Point2 {
        let (x, y) = (real(&p.0), real(&p.1));
        let cx = &self.c * &x;
        let sy = &self.s * &y;
        let sx = &self.s * &x;
        let cy = &self.c * &y;
        let px = &(&cx - &sy) + &self.tx;
        let py = &(&sx + &cy) + &self.ty;
        Point2::new(px, py)
    }

    fn points(&self, points: &[P]) -> Vec<Point2> {
        points.iter().map(|p| self.point(p)).collect()
    }
}

// ---------------------------------------------------------------------------
// Template shapes.

type P = (Rational, Rational);

fn pt(x: i64, y: i64) -> P {
    (int(x), int(y))
}

fn shift(p: &P, dx: &Rational, dy: &Rational) -> P {
    (&p.0 + dx, &p.1 + dy)
}

#[derive(Clone, Debug)]
enum Shape {
    Line(P, P),
    Arc(P, P, Rational),
    ArcCenter(P, P, P, bool),
    Quadratic([P; 3]),
    Cubic([P; 4]),
    Conic([P; 3], [Rational; 3]),
    RationalBezier(Vec<P>, Vec<Rational>),
    Spline(usize, Vec<P>, Vec<Rational>),
    Nurbs(usize, Vec<P>, Vec<Rational>, Vec<Rational>),
}

impl Shape {
    fn geometry(&self, f: &Frame) -> Option<CurveGeometry2> {
        Some(match self {
            Shape::Line(a, b) => {
                CurveGeometry2::Line(LineSeg2::try_new(f.point(a), f.point(b)).ok()?)
            }
            Shape::Arc(a, b, bulge) => CurveGeometry2::CircularArc(
                CircularArc2::from_bulge(f.point(a), f.point(b), real(bulge)).ok()?,
            ),
            Shape::ArcCenter(a, b, c, clockwise) => CurveGeometry2::CircularArc(
                CircularArc2::try_from_center(f.point(a), f.point(b), f.point(c), *clockwise)
                    .ok()?,
            ),
            Shape::Quadratic([a, b, c]) => CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(
                f.point(a),
                f.point(b),
                f.point(c),
            )),
            Shape::Cubic([a, b, c, d]) => CurveGeometry2::CubicBezier(CubicBezier2::new(
                f.point(a),
                f.point(b),
                f.point(c),
                f.point(d),
            )),
            Shape::Conic([a, b, c], [wa, wb, wc]) => CurveGeometry2::RationalQuadraticBezier(
                RationalQuadraticBezier2::try_new(
                    f.point(a),
                    f.point(b),
                    f.point(c),
                    real(wa),
                    real(wb),
                    real(wc),
                )
                .ok()?,
            ),
            Shape::RationalBezier(points, weights) => CurveGeometry2::RationalBezier(
                RationalBezier2::try_new(f.points(points), reals(weights)).ok()?,
            ),
            Shape::Spline(degree, points, knots) => CurveGeometry2::PolynomialBSpline(
                PolynomialSplineCurve2::try_new(*degree, f.points(points), reals(knots)).ok()?,
            ),
            Shape::Nurbs(degree, points, weights, knots) => CurveGeometry2::Nurbs(
                NurbsCurve2::try_new(*degree, f.points(points), reals(weights), reals(knots))
                    .ok()?,
            ),
        })
    }

    fn curve(&self, f: &Frame) -> Option<Curve2> {
        self.geometry(f).map(Curve2::new)
    }

    fn segment(&self, f: &Frame) -> Option<Segment2> {
        match self.geometry(f)? {
            CurveGeometry2::Line(line) => Some(Segment2::Line(line)),
            CurveGeometry2::CircularArc(arc) => Some(Segment2::Arc(arc)),
            _ => None,
        }
    }

    fn nurbs(&self, f: &Frame) -> Option<NurbsCurve2> {
        match self.geometry(f)? {
            CurveGeometry2::Nurbs(nurbs) => Some(nurbs),
            _ => None,
        }
    }
}

fn clamped(degree: usize, spans: usize) -> Vec<Rational> {
    let mut knots = vec![int(0); degree + 1];
    knots.extend((1..spans).map(|i| ratio(i as i64, spans as u64)));
    knots.extend(vec![int(1); degree + 1]);
    knots
}

fn s_cubic() -> Shape {
    Shape::Cubic([pt(-4, 0), pt(-1, 8), pt(1, -8), pt(4, 0)])
}

fn wave_cubic() -> Shape {
    Shape::Cubic([pt(-4, 1), pt(-1, -3), pt(1, 3), pt(4, -1)])
}

fn loop_cubic() -> Shape {
    Shape::Cubic([pt(-2, 0), pt(4, 4), pt(-4, 4), pt(2, 0)])
}

fn rational_cubic() -> Shape {
    Shape::RationalBezier(
        vec![pt(-4, 0), pt(-1, 8), pt(1, -8), pt(4, 0)],
        vec![int(1), int(2), int(3), int(1)],
    )
}

fn arc_family() -> Shape {
    Shape::Arc(pt(-4, -1), pt(4, 1), ratio(1, 2))
}

/// A cubic NURBS zigzag with `spans` spans and alternating weights.
fn nurbs_zigzag(spans: usize, sign: i64) -> Shape {
    let count = spans + 3;
    let points = (0..count)
        .map(|i| {
            let y = if i % 2 == 0 { 3 } else { -3 };
            pt(2 * i as i64 - count as i64, sign * y)
        })
        .collect();
    let weights = (0..count).map(|i| int(1 + (i % 2) as i64)).collect();
    Shape::Nurbs(3, points, weights, clamped(3, spans))
}

/// Degree-`degree` zigzag with `spans` spans (`degree + spans` controls).
fn zigzag_points(count: usize) -> Vec<P> {
    (0..count)
        .map(|i| pt(2 * i as i64 - count as i64, if i % 2 == 0 { 2 } else { -2 }))
        .collect()
}

fn degree_shape(kind: &str, degree: usize) -> Shape {
    match kind {
        "Bézier (polynomial)" => {
            Shape::Spline(degree, zigzag_points(degree + 1), clamped(degree, 1))
        }
        "rational Bézier" => Shape::RationalBezier(
            zigzag_points(degree + 1),
            (0..=degree).map(|i| int(1 + (i % 3) as i64)).collect(),
        ),
        _ => {
            let count = degree + 4;
            Shape::Nurbs(
                degree,
                zigzag_points(count),
                (0..count).map(|i| int(1 + (i % 2) as i64)).collect(),
                clamped(degree, 4),
            )
        }
    }
}

const DEGREE_KINDS: [&str; 3] = ["Bézier (polynomial)", "rational Bézier", "NURBS (4 spans)"];

fn family_shape(kind: &str) -> Shape {
    match kind {
        "line" => Shape::Line(pt(-4, -1), pt(4, 1)),
        "arc" => arc_family(),
        "cubic Bézier" => s_cubic(),
        "rational cubic" => rational_cubic(),
        _ => nurbs_zigzag(4, 1),
    }
}

const FAMILIES: [&str; 4] = ["arc", "cubic Bézier", "rational cubic", "NURBS"];

/// Cubic with a single minimum at the origin, y = x²/6 + x³/72 on [-6, 6];
/// radius of curvature 3 there.
fn valley_cubic() -> [P; 4] {
    [pt(-6, 3), pt(-2, 1), pt(2, -5), pt(6, 9)]
}

/// Bernstein controls of y = x^m over x in [-1, 1].
fn power_controls(m: usize, sign: i64) -> Vec<P> {
    (0..=m)
        .map(|i| {
            let x = ratio(2 * i as i64 - m as i64, m as u64);
            let y = if (m - i).is_multiple_of(2) {
                sign
            } else {
                -sign
            };
            (x, int(y))
        })
        .collect()
}

/// Vertices of a regular `n`-gon of radius 2^24 rounded to integers, with
/// angular phase `phase` (in steps).
fn polygon(n: usize, phase: f64, radius: f64) -> Vec<P> {
    (0..n)
        .map(|i| {
            let theta = 2.0 * PI * (i as f64 + phase) / n as f64;
            pt(
                (radius * theta.cos()).round() as i64,
                (radius * theta.sin()).round() as i64,
            )
        })
        .collect()
}

const RADIUS: f64 = 16_777_216.0;

/// Closed loop of `kind` curves through regular polygon vertices: line
/// edges, outward arcs, tangent-continuous quadratics or circle-like cubics.
fn closed_loop(kind: &str, n: usize, phase: f64) -> Vec<Shape> {
    let vertices = polygon(n, phase, RADIUS);
    let half = PI / n as f64;
    (0..n)
        .map(|i| {
            let a = vertices[i].clone();
            let b = vertices[(i + 1) % n].clone();
            let mid = 2.0 * PI * (i as f64 + phase + 0.5) / n as f64;
            match kind {
                "lines" => Shape::Line(a, b),
                "arcs" => Shape::Arc(a, b, ratio(1, 8)),
                "quadratic Béziers" => {
                    let r = RADIUS / half.cos();
                    let c = pt(
                        (r * mid.cos()).round() as i64,
                        (r * mid.sin()).round() as i64,
                    );
                    Shape::Quadratic([a, c, b])
                }
                _ => {
                    let h = 4.0 / 3.0 * (half / 2.0).tan() * RADIUS;
                    let ta = 2.0 * PI * (i as f64 + phase) / n as f64;
                    let tb = 2.0 * PI * (i as f64 + 1.0 + phase) / n as f64;
                    let c1 = pt(
                        (RADIUS * ta.cos() - h * ta.sin()).round() as i64,
                        (RADIUS * ta.sin() + h * ta.cos()).round() as i64,
                    );
                    let c2 = pt(
                        (RADIUS * tb.cos() + h * tb.sin()).round() as i64,
                        (RADIUS * tb.sin() - h * tb.cos()).round() as i64,
                    );
                    Shape::Cubic([a, c1, c2, b])
                }
            }
        })
        .collect()
}

const LOOP_KINDS: [&str; 4] = ["lines", "arcs", "quadratic Béziers", "cubic Béziers"];

fn region_from(shapes: &[Shape], f: &Frame) -> Option<CurveRegion2> {
    let curves = shapes
        .iter()
        .map(|s| s.curve(f))
        .collect::<Option<Vec<_>>>()?;
    let path = CurvePath2::try_new(curves).ok()?;
    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).ok()
}

fn path_from(shapes: &[Shape], f: &Frame) -> Option<CurvePath2> {
    let curves = shapes
        .iter()
        .map(|s| s.curve(f))
        .collect::<Option<Vec<_>>>()?;
    CurvePath2::try_new(curves).ok()
}

/// Star polygon {n/k} with k = n/2 - 1, whose edges cross O(n²) times.
fn star(n: usize) -> Vec<Shape> {
    let vertices = polygon(n, 0.0, RADIUS);
    let step = (n / 2).saturating_sub(1).max(1);
    (0..n)
        .map(|i| {
            Shape::Line(
                vertices[(i * step) % n].clone(),
                vertices[((i + 1) * step) % n].clone(),
            )
        })
        .collect()
}

/// Star-shaped random polygon with radii in [R/2, R].
fn random_polygon(n: usize, rng: &mut Rng, dx: i64) -> Vec<Shape> {
    let vertices: Vec<P> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * i as f64 / n as f64;
            let r = RADIUS * (0.5 + 0.5 * rng.unit());
            pt(
                (r * theta.cos()).round() as i64 + dx,
                (r * theta.sin()).round() as i64,
            )
        })
        .collect();
    (0..n)
        .map(|i| Shape::Line(vertices[i].clone(), vertices[(i + 1) % n].clone()))
        .collect()
}

fn zigzag_string(n: usize, inverted: bool, dy: i64, bulge: &Rational) -> Option<CurveString2> {
    let vertices: Vec<BulgeVertex2> = (0..=n)
        .map(|i| {
            let up = (i % 2 == 1) != inverted;
            BulgeVertex2::new(
                Point2::new(Real::from(i as i32), Real::from(dy as i32 + i32::from(up))),
                real(bulge),
            )
        })
        .collect();
    CurveString2::from_bulge_vertices(&vertices).ok()
}

fn intersect(a: &Shape, b: &Shape, embed: &Embed) -> Option<usize> {
    let f = embed.frame();
    let (a, b) = (a.curve(&f)?, b.curve(&f)?);
    guarded(|| a.intersect_curve(&b))
        .filter(|r| r.is_complete())
        .map(|r| r.contacts().len() + r.overlaps().len())
}

fn locations(curve: &Curve2, point: &CurvePoint2) -> Option<usize> {
    match curve.point_locations(point).ok()? {
        CurvePointLocations2::EntireCurve => Some(usize::MAX),
        CurvePointLocations2::Locations(found) => Some(found.len()),
        CurvePointLocations2::Components(found) => Some(found.len()),
    }
}

/// An on-curve point at `t`, detached from the curve's own evaluation evidence.
fn on_curve(curve: &Curve2, t: &Rational) -> Option<CurvePoint2> {
    let point = curve.point_at(&CurveParameter2::from(real(t))).ok()?;
    Some(match point.coordinates() {
        Some(p) => CurvePoint2::from(p.clone()),
        None => point,
    })
}

fn parameter(t: &Rational) -> CurveParameter2 {
    CurveParameter2::from(real(t))
}

// ---------------------------------------------------------------------------

fn main() {
    let mut atlas = Atlas::from_args("hypercurve");
    start_watchdog(atlas.config().single_ceiling * 2);

    let bits = powers_of_two(0, 10);
    let few_bits = powers_of_two(0, 8);
    let degrees: Vec<f64> = [1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 20, 24, 32]
        .map(f64::from)
        .to_vec();
    let segments = powers_of_two(2, 10);

    construction(&mut atlas, &bits, &degrees);
    evaluation(&mut atlas, &bits, &degrees);
    intersection(&mut atlas, &bits, &few_bits);
    location(&mut atlas, &bits, &degrees);
    offsets(&mut atlas, &segments);
    regions(&mut atlas, &segments);
    weights(&mut atlas);
    splines(&mut atlas, &degrees);
    projection(&mut atlas);
    #[cfg(feature = "svg")]
    svg(&mut atlas, &segments);
    atlas.finish();
}

fn construction(atlas: &mut Atlas, bits: &[f64], degrees: &[f64]) {
    atlas.operation(
        "construction",
        "construct and validate",
        "coordinate bits",
        "Build exact points from rationals and construct a validated Curve2 (cold); the baseline included in every cold operation.",
    );
    for kind in ["line", "arc", "cubic Bézier", "NURBS"] {
        let shape = family_shape(kind);
        atlas.sweep(
            kind,
            bits,
            |m, rng| Embed::rational(m as u32, rng),
            |embed| shape.curve(&embed.frame()).map(|c| c.is_periodic()),
        );
    }
    atlas.fuzz(
        "mixed family and representation",
        &powers_of_two(0, 8),
        |m, rng| {
            let kind =
                ["line", "arc", "cubic Bézier", "rational cubic", "NURBS"][rng.below(5) as usize];
            let num = NUMS[rng.below(3) as usize].1;
            let size = rng.range(1, m as i64) as u32;
            (family_shape(kind), Embed::random(num, size, rng))
        },
        |(shape, embed)| shape.curve(&embed.frame()).map(|c| c.is_periodic()),
    );

    atlas.operation(
        "construction",
        "construct by degree",
        "curve degree",
        "Construct and validate a zigzag curve of growing degree with 16-bit rational coordinates (NURBS: 4 spans, clamped knots).",
    );
    for kind in DEGREE_KINDS {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| (degree_shape(kind, d as usize), Embed::rational(16, rng)),
            |(shape, embed)| shape.curve(&embed.frame()).map(|c| c.is_periodic()),
        );
    }
}

fn evaluation(atlas: &mut Atlas, bits: &[f64], degrees: &[f64]) {
    let warm_point = |shape: &Shape, embed: &Embed, rng: &mut Rng| {
        let curve = shape.curve(&embed.frame()).expect("template curve");
        (curve, parameter(&unit_parameter(rng)))
    };

    atlas.operation(
        "evaluation",
        "point_at by representation",
        "coordinate bits",
        "Evaluate a prebuilt cubic Bézier (warm) at a 30-bit rational parameter and approximate the exact point to f64.",
    );
    for (kind, num) in NUMS {
        atlas.sweep(
            kind,
            bits,
            |m, rng| warm_point(&s_cubic(), &Embed::random(num, m as u32, rng), rng),
            |(curve, t)| curve.point_at(t).ok().and_then(|p| force(&p)),
        );
    }

    atlas.operation(
        "evaluation",
        "point_at by family",
        "coordinate bits",
        "Evaluate a prebuilt curve (warm, rational coordinates) at a 30-bit rational parameter and approximate the point to f64.",
    );
    for kind in FAMILIES {
        atlas.sweep(
            kind,
            bits,
            |m, rng| warm_point(&family_shape(kind), &Embed::rational(m as u32, rng), rng),
            |(curve, t)| curve.point_at(t).ok().and_then(|p| force(&p)),
        );
    }
    atlas.fuzz(
        "random family, representation and parameter",
        &powers_of_two(0, 8),
        |m, rng| {
            let kind = FAMILIES[rng.below(4) as usize];
            let num = NUMS[rng.below(3) as usize].1;
            let size = rng.range(1, m as i64) as u32;
            warm_point(&family_shape(kind), &Embed::random(num, size, rng), rng)
        },
        |(curve, t)| curve.point_at(t).ok().and_then(|p| force(&p)),
    );

    atlas.operation(
        "evaluation",
        "point_at by degree",
        "curve degree",
        "Evaluate a prebuilt zigzag curve of growing degree (warm, 16-bit rational coordinates) and approximate the point.",
    );
    for kind in DEGREE_KINDS {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| {
                warm_point(
                    &degree_shape(kind, d as usize),
                    &Embed::rational(16, rng),
                    rng,
                )
            },
            |(curve, t)| curve.point_at(t).ok().and_then(|p| force(&p)),
        );
    }
    atlas.grid(
        "NURBS (4 spans)",
        ("curve degree", &[1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0]),
        ("coordinate bits", &[1.0, 4.0, 16.0, 64.0, 256.0, 1024.0]),
        |d, b, rng| {
            warm_point(
                &degree_shape("NURBS", d as usize),
                &Embed::rational(b as u32, rng),
                rng,
            )
        },
        |(curve, t)| curve.point_at(t).ok().and_then(|p| force(&p)),
    );

    atlas.operation(
        "evaluation",
        "derivative_at by degree",
        "curve degree",
        "First derivative of a prebuilt zigzag curve of growing degree (warm) at a 30-bit rational parameter, approximated to f64.",
    );
    for kind in DEGREE_KINDS {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| {
                warm_point(
                    &degree_shape(kind, d as usize),
                    &Embed::rational(16, rng),
                    rng,
                )
            },
            |(curve, t)| {
                let v = curve.derivative_at(t).ok()?;
                let (dx, dy) = v.represented_coordinates()?;
                Some(approximate(dx)? + approximate(dy)?)
            },
        );
    }

    atlas.operation(
        "bounds",
        "bounds (cold)",
        "coordinate bits",
        "Rebuild the curve and compute its cached conservative enclosure (control hull for polynomial spans, exact extrema for arcs and rational spans).",
    );
    for kind in FAMILIES {
        let shape = family_shape(kind);
        atlas.sweep(
            kind,
            bits,
            |m, rng| Embed::rational(m as u32, rng),
            |embed| {
                let curve = shape.curve(&embed.frame())?;
                let b = curve.bounds().ok()?;
                Some(b.min_x().to_f64_lossy()? + b.max_y().to_f64_lossy()?)
            },
        );
    }
    atlas.sweep(
        "cubic Bézier, irrational",
        bits,
        |m, rng| Embed::random(Num::Irrational, m as u32, rng),
        |embed| {
            let curve = s_cubic().curve(&embed.frame())?;
            let b = curve.bounds().ok()?;
            Some(b.min_x().to_f64_lossy()? + b.max_y().to_f64_lossy()?)
        },
    );

    atlas.operation(
        "subdivision",
        "split_at (cold)",
        "coordinate bits",
        "Rebuild the curve and split it exactly at a 30-bit rational parameter.",
    );
    for kind in FAMILIES {
        let shape = family_shape(kind);
        atlas.sweep(
            kind,
            bits,
            |m, rng| {
                (
                    Embed::rational(m as u32, rng),
                    parameter(&unit_parameter(rng)),
                )
            },
            |(embed, t)| {
                let curve = shape.curve(&embed.frame())?;
                curve.split_at(t.clone()).ok().map(|(a, _)| a.is_periodic())
            },
        );
    }
}

fn intersection(atlas: &mut Atlas, bits: &[f64], few_bits: &[f64]) {
    let pairs: [(&str, Shape, Shape); 4] = [
        ("line × cubic", Shape::Line(pt(-5, -1), pt(5, 1)), s_cubic()),
        ("cubic × cubic", s_cubic(), wave_cubic()),
        ("arc × cubic", arc_family(), s_cubic()),
        ("NURBS × NURBS", nurbs_zigzag(4, 1), nurbs_zigzag(3, -1)),
    ];
    atlas.operation(
        "intersection",
        "intersect_curve by family",
        "coordinate bits",
        "Exact contacts between two transversally crossing curves (cold), placed by a shared random rational similarity. Incomplete results (blockers) are errors.",
    );
    for (kind, a, b) in &pairs {
        atlas.sweep(
            kind,
            bits,
            |m, rng| Embed::rational(m as u32, rng),
            |embed| intersect(a, b, embed),
        );
    }
    atlas.fuzz(
        "random cubic pairs",
        &powers_of_two(0, 7),
        |m, rng| {
            let mut cubic = || {
                let mut p = || pt(rng.range(-8, 8), rng.range(-8, 8));
                Shape::Cubic([p(), p(), p(), p()])
            };
            let (a, b) = (cubic(), cubic());
            let size = rng.range(1, m as i64) as u32;
            (a, b, Embed::rational(size, rng))
        },
        |(a, b, embed)| intersect(a, b, embed),
    );

    atlas.operation(
        "intersection",
        "intersect_curve by representation",
        "coordinate bits",
        "Exact contacts between two crossing cubic Béziers (cold) under integer, rational and irrational placements.",
    );
    for (kind, num) in NUMS {
        atlas.sweep(
            kind,
            bits,
            |m, rng| Embed::random(num, m as u32, rng),
            |embed| intersect(&s_cubic(), &wave_cubic(), embed),
        );
    }

    // Near tangency: separations of 2^-k around a curvature-3 valley.
    let valley = valley_cubic();
    let near = move |kind: &str, k: i32| -> (Shape, Shape) {
        let eps = pow2(-k);
        let zero = int(0);
        match kind {
            "line × cubic, crossing (+2^-k)" => (
                Shape::Line((int(-7), eps.clone()), (int(7), eps)),
                Shape::Cubic(valley.clone()),
            ),
            "line × cubic, near miss (−2^-k)" => (
                Shape::Line((int(-7), &zero - &eps), (int(7), &zero - &eps)),
                Shape::Cubic(valley.clone()),
            ),
            "cubic × mirrored cubic, crossing" => {
                let mirrored = valley
                    .clone()
                    .map(|p| shift(&(p.0.clone(), &zero - &p.1), &zero, &eps));
                (Shape::Cubic(valley.clone()), Shape::Cubic(mirrored))
            }
            _ => {
                let cy = &int(2) + &eps;
                (
                    Shape::ArcCenter(pt(1, 0), pt(-1, 0), pt(0, 0), false),
                    Shape::ArcCenter(
                        (int(-1), cy.clone()),
                        (int(1), cy.clone()),
                        (int(0), cy),
                        false,
                    ),
                )
            }
        }
    };
    const NEAR_KINDS: [&str; 4] = [
        "line × cubic, crossing (+2^-k)",
        "line × cubic, near miss (−2^-k)",
        "cubic × mirrored cubic, crossing",
        "arc × arc, gap 2^-k",
    ];
    let separations = powers_of_two(0, 11);
    atlas.operation(
        "intersection",
        "near-tangency",
        "separation exponent k (gap 2^-k)",
        "Curves separated by 2^-k near a tangency (radius of curvature 3; unit circles): two crossings that merge as k grows, or a near miss that must be proven disjoint. Exact rational coordinates, cold.",
    );
    for kind in NEAR_KINDS {
        atlas.sweep(
            kind,
            &separations,
            |k, _| (near(kind, k as i32), Embed::identity()),
            |((a, b), embed)| intersect(a, b, embed),
        );
    }
    atlas.fuzz(
        "line × cubic, random sign and jittered k",
        &separations,
        |k, rng| {
            let kind = NEAR_KINDS[rng.below(2) as usize];
            let k = (k * (0.75 + rng.unit() / 2.0)).round() as i32;
            (near(kind, k), Embed::identity())
        },
        |((a, b), embed)| intersect(a, b, embed),
    );
    atlas.grid(
        "line × cubic, crossing (+2^-k)",
        (
            "separation exponent k",
            &[1.0, 4.0, 16.0, 64.0, 256.0, 1024.0],
        ),
        ("coordinate bits", few_bits),
        |k, b, rng| {
            (
                near("line × cubic, crossing (+2^-k)", k as i32),
                Embed::rational(b as u32, rng),
            )
        },
        |((a, b), embed)| intersect(a, b, embed),
    );

    atlas.operation(
        "intersection",
        "intersection multiplicity",
        "contact multiplicity m",
        "Contact of order m at the origin: y=0 against y=x^m (degree-m Bézier) and y=x^m against y=−x^m. Exact rational coordinates, cold.",
    );
    let axis = Shape::Line(pt(-2, 0), pt(2, 0));
    let multiplicities: Vec<f64> = (1..=20).map(f64::from).collect();
    atlas.sweep(
        "line × x^m (polynomial Bézier)",
        &multiplicities,
        |m, _| {
            Shape::Spline(
                m as usize,
                power_controls(m as usize, 1),
                clamped(m as usize, 1),
            )
        },
        |curve| intersect(&axis, curve, &Embed::identity()),
    );
    atlas.sweep(
        "line × x^m (rational Bézier, unit weights)",
        &multiplicities,
        |m, _| Shape::RationalBezier(power_controls(m as usize, 1), vec![int(1); m as usize + 1]),
        |curve| intersect(&axis, curve, &Embed::identity()),
    );
    atlas.sweep(
        "x^m × −x^m (polynomial Béziers)",
        &multiplicities,
        |m, _| {
            let m = m as usize;
            (
                Shape::Spline(m, power_controls(m, 1), clamped(m, 1)),
                Shape::Spline(m, power_controls(m, -1), clamped(m, 1)),
            )
        },
        |(a, b)| intersect(a, b, &Embed::identity()),
    );

    atlas.operation(
        "intersection",
        "self_intersections",
        "coordinate bits",
        "Exact off-diagonal self-contacts of one curve (cold): a looped cubic, an S-shaped cubic without a loop and a looped rational cubic.",
    );
    let selves = [
        ("cubic with loop", loop_cubic()),
        ("S cubic, no loop", s_cubic()),
        (
            "rational cubic with loop",
            Shape::RationalBezier(
                vec![pt(-2, 0), pt(4, 4), pt(-4, 4), pt(2, 0)],
                vec![int(1), int(2), int(2), int(1)],
            ),
        ),
    ];
    for (kind, shape) in &selves {
        atlas.sweep(
            kind,
            bits,
            |m, rng| Embed::rational(m as u32, rng),
            |embed| {
                let curve = shape.curve(&embed.frame())?;
                guarded(|| curve.self_intersections())
                    .filter(|r| r.is_complete())
                    .map(|r| r.contacts().len())
            },
        );
    }

    atlas.operation(
        "intersection",
        "intersect_curve_string",
        "segments per string",
        "Line/arc strings of n segments (cold): separated zigzags, inverted zigzags crossing once per segment, and the same with arcs (bulge 1/4).",
    );
    let segments = powers_of_two(1, 10);
    for (kind, dy, bulge) in [
        ("separated line zigzags", 100, int(0)),
        ("crossing line zigzags", 0, int(0)),
        ("crossing arc zigzags", 0, ratio(1, 4)),
    ] {
        atlas.sweep(
            kind,
            &segments,
            |n, _| n as usize,
            |&n| {
                let a = zigzag_string(n, false, 0, &bulge)?;
                let b = zigzag_string(n, true, dy, &bulge)?;
                guarded(|| a.intersect_curve_string(&b)).map(|r| r.len())
            },
        );
    }
}

fn location(atlas: &mut Atlas, bits: &[f64], degrees: &[f64]) {
    let make = |shape: &Shape, embed: &Embed, rng: &mut Rng| {
        let curve = shape.curve(&embed.frame()).expect("template curve");
        let point = on_curve(&curve, &unit_parameter(rng)).expect("on-curve point");
        (curve, point)
    };
    let found =
        |(curve, point): &(Curve2, CurvePoint2)| locations(curve, point).filter(|&count| count > 0);

    atlas.operation(
        "parameter location",
        "point_locations by family",
        "coordinate bits",
        "Every exact parameter at which a prebuilt curve (warm, rational coordinates) passes through one of its points given by coordinates. An empty answer is an error.",
    );
    for kind in FAMILIES {
        atlas.sweep(
            kind,
            bits,
            |m, rng| make(&family_shape(kind), &Embed::rational(m as u32, rng), rng),
            found,
        );
    }
    atlas.fuzz(
        "random family and representation",
        &powers_of_two(0, 7),
        |m, rng| {
            let kind = FAMILIES[rng.below(4) as usize];
            let num = NUMS[rng.below(3) as usize].1;
            let size = rng.range(1, m as i64) as u32;
            make(&family_shape(kind), &Embed::random(num, size, rng), rng)
        },
        found,
    );

    atlas.operation(
        "parameter location",
        "point_locations by representation",
        "coordinate bits",
        "Locate an on-curve point of a prebuilt cubic Bézier (warm) under integer, rational and irrational placements.",
    );
    for (kind, num) in NUMS {
        atlas.sweep(
            kind,
            bits,
            |m, rng| make(&s_cubic(), &Embed::random(num, m as u32, rng), rng),
            found,
        );
    }

    atlas.operation(
        "parameter location",
        "point_locations by degree",
        "curve degree",
        "Locate an on-curve point of a prebuilt zigzag curve of growing degree (warm, 16-bit rational coordinates).",
    );
    for kind in DEGREE_KINDS {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| {
                make(
                    &degree_shape(kind, d as usize),
                    &Embed::rational(16, rng),
                    rng,
                )
            },
            found,
        );
    }

    atlas.operation(
        "parameter location",
        "point_locations near miss",
        "offset exponent k (point 2^-k off the curve)",
        "Locate a point displaced vertically by 2^-k from a prebuilt curve (warm, rational coordinates); the exact answer is empty.",
    );
    for kind in ["cubic Bézier", "rational cubic", "NURBS"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 11),
            |k, rng| {
                let curve = family_shape(kind)
                    .curve(&Embed::identity().frame())
                    .unwrap();
                let point = on_curve(&curve, &unit_parameter(rng)).unwrap();
                let p = point.coordinates().unwrap();
                let moved = Point2::new(p.x().clone(), p.y() + &real(&pow2(-(k as i32))));
                (curve, CurvePoint2::from(moved))
            },
            |(curve, point)| locations(curve, point),
        );
    }
}

fn offsets(atlas: &mut Atlas, segments: &[f64]) {
    // Distance ratios d/ρ from 2^-6 to 2^2 in half-octave steps.
    let ratios: Vec<f64> = (-12..=4).map(|h| 2f64.powf(f64::from(h) / 2.0)).collect();
    let exact_ratio = |r: f64| -> Rational {
        // Dyadic with 16 fractional bits; exact for every ladder value below.
        ratio((r * 65_536.0).round() as i64, 65_536)
    };

    atlas.operation(
        "offset",
        "Bézier parallel and cusp analysis",
        "offset distance / radius of curvature",
        "Exact procedural parallel of a Bézier at distance d, then certified cusp isolation over the unit range. Concave-side parallels develop cusps once d exceeds the minimum radius of curvature ρ (ratio 1).",
    );
    let parabola = [pt(-2, 4), pt(0, -4), pt(2, 4)];
    let valley = valley_cubic();
    for (kind, rho, left) in [
        ("quadratic, concave side (ρ=1/2)", ratio(1, 2), true),
        ("cubic, concave side (ρ=3)", int(3), true),
        ("cubic, convex side (no cusps)", int(3), false),
    ] {
        let parabola = parabola.clone();
        let valley = valley.clone();
        atlas.sweep(
            kind,
            &ratios,
            |r, _| &exact_ratio(r) * &rho,
            |d| {
                let f = Embed::identity().frame();
                let unit = CurveParameterRange2::unit();
                let distance = real(d);
                guarded(|| {
                    let parallel = if kind.starts_with("quadratic") {
                        let [a, b, c] = &parabola;
                        QuadraticBezier2::new(f.point(a), f.point(b), f.point(c))
                            .parallel_left(distance)
                    } else {
                        let [a, b, c, e] = &valley;
                        let cubic =
                            CubicBezier2::new(f.point(a), f.point(b), f.point(c), f.point(e));
                        if left {
                            cubic.parallel_left(distance)
                        } else {
                            cubic.parallel_right(distance)
                        }
                    }
                    .map_err(drop)?;
                    parallel.singularity_analysis(&unit).map_err(drop)
                })
                .map(|analysis| analysis.parallel_cusps().len())
            },
        );
    }

    // Regions for offsetting: parabolic cap {x² ≤ y ≤ 4} (ρ=1/2 at the vertex),
    // unit disk and square of inradius 1.
    let cap = vec![
        Shape::Quadratic([pt(-2, 4), pt(0, -4), pt(2, 4)]),
        Shape::Line(pt(2, 4), pt(-2, 4)),
    ];
    let disk = vec![
        Shape::Arc(pt(-1, 0), pt(1, 0), int(1)),
        Shape::Arc(pt(1, 0), pt(-1, 0), int(1)),
    ];
    let square = vec![
        Shape::Line(pt(-1, -1), pt(1, -1)),
        Shape::Line(pt(1, -1), pt(1, 1)),
        Shape::Line(pt(1, 1), pt(-1, 1)),
        Shape::Line(pt(-1, 1), pt(-1, -1)),
    ];
    let offset = |region: &CurveRegion2, distance: &Rational, style: &OffsetCornerStyle2| {
        guarded(|| region.offset(real(distance), style)).map(|r| r.boundary_loops().len())
    };
    atlas.operation(
        "offset",
        "region offset by distance",
        "offset distance / feature size",
        "Exact regularized region offset of a prebuilt region (round joins unless noted). Feature size: radius of curvature 1/2 at the parabolic cap's vertex, the unit disk's radius, the square's inradius. Inward offsets collapse or develop trimmed cusps near ratio 1.",
    );
    let identity = Embed::identity();
    for (kind, shapes, size, sign, style) in [
        (
            "parabolic cap, inward",
            &cap,
            ratio(1, 2),
            -1,
            OffsetCornerStyle2::Round,
        ),
        (
            "parabolic cap, outward",
            &cap,
            ratio(1, 2),
            1,
            OffsetCornerStyle2::Round,
        ),
        (
            "unit disk, inward",
            &disk,
            int(1),
            -1,
            OffsetCornerStyle2::Round,
        ),
        (
            "square, inward (miter 4)",
            &square,
            int(1),
            -1,
            OffsetCornerStyle2::Miter {
                limit: Real::from(4),
            },
        ),
    ] {
        let region = region_from(shapes, &identity.frame());
        atlas.sweep(
            kind,
            &ratios,
            |r, _| &(&exact_ratio(r) * &size) * &int(sign),
            |d| offset(region.as_ref()?, d, &style),
        );
    }
    atlas.fuzz(
        "parabolic cap, random signed ratio",
        &ratios,
        |r, rng| {
            let jitter = 1.0 + (rng.unit() - 0.5) / 4.0;
            let sign = if rng.coin() { 1 } else { -1 };
            &(&exact_ratio(r * jitter) * &ratio(1, 2)) * &int(sign)
        },
        {
            let region = region_from(&cap, &identity.frame());
            move |d| offset(region.as_ref()?, d, &OffsetCornerStyle2::Round)
        },
    );

    atlas.operation(
        "offset",
        "region offset by representation",
        "coordinate bits",
        "Inward round offset of the parabolic cap by a quarter of its vertex radius of curvature, under integer, rational and irrational placements (prebuilt region).",
    );
    for (kind, num) in NUMS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |m, rng| {
                let embed = Embed::random(num, m as u32, rng);
                let region = region_from(&cap, &embed.frame());
                let d = &int(0) - &embed.length(&ratio(1, 8));
                (region, d)
            },
            |(region, d)| offset(region.as_ref()?, d, &OffsetCornerStyle2::Round),
        );
    }
    atlas.grid(
        "parabolic cap, inward",
        (
            "offset distance / radius of curvature",
            &[0.125, 0.5, 1.0, 2.0, 4.0],
        ),
        ("coordinate bits", &[1.0, 4.0, 16.0, 64.0]),
        |r, b, rng| {
            let embed = Embed::rational(b as u32, rng);
            let region = region_from(&cap, &embed.frame());
            let d = &int(0) - &embed.length(&(&exact_ratio(r) * &ratio(1, 2)));
            (region, d)
        },
        |(region, d)| offset(region.as_ref()?, d, &OffsetCornerStyle2::Round),
    );

    atlas.operation(
        "offset",
        "stroke_path",
        "segments in the path",
        "Exact stroke region (half-width 1/4) of a prebuilt open zigzag line path with n right-angle corners; overlapping joins go through the authoritative arrangement.",
    );
    for (kind, style, cap) in [
        (
            "round joins, round caps",
            OffsetCornerStyle2::Round,
            OffsetCap::Round,
        ),
        (
            "miter joins (limit 4), butt caps",
            OffsetCornerStyle2::Miter {
                limit: Real::from(4),
            },
            OffsetCap::Butt,
        ),
        (
            "bevel joins, square caps",
            OffsetCornerStyle2::Bevel,
            OffsetCap::Square,
        ),
    ] {
        atlas.sweep(
            kind,
            segments,
            |n, _| {
                let n = n as usize;
                let shapes: Vec<Shape> = (0..n)
                    .map(|i| {
                        Shape::Line(
                            pt(i as i64, (i % 2) as i64),
                            pt(i as i64 + 1, ((i + 1) % 2) as i64),
                        )
                    })
                    .collect();
                path_from(&shapes, &Embed::identity().frame())
            },
            |path| {
                let path = path.as_ref()?;
                guarded(|| CurveRegion2::stroke_path(path, real(&ratio(1, 4)), &style, cap))
                    .map(|r| r.boundary_loops().len())
            },
        );
    }
}

fn regions(atlas: &mut Atlas, segments: &[f64]) {
    let identity = Embed::identity();
    atlas.operation(
        "regions",
        "region from boundary path",
        "boundary segments",
        "Arrange and regularize one closed boundary path (cold) into a CurveRegion2 under NonZero: convex polygons, a self-crossing {n/(n/2−1)} star with O(n²) crossings, arc polygons and tangent-continuous quadratic loops (25-bit integer vertices).",
    );
    for (kind, shapes) in [
        ("convex polygon (lines)", None),
        ("self-crossing star (lines)", Some("star")),
        ("arc polygon", Some("arcs")),
        ("quadratic Bézier loop", Some("quadratic Béziers")),
    ] {
        let frame_embed = identity.clone();
        atlas.sweep(
            kind,
            segments,
            |n, _| match shapes {
                None => closed_loop("lines", n as usize, 0.0),
                Some("star") => star(n as usize),
                Some(kind) => closed_loop(kind, n as usize, 0.0),
            },
            |shapes| {
                let f = frame_embed.frame();
                let curves = shapes
                    .iter()
                    .map(|s| s.curve(&f))
                    .collect::<Option<Vec<_>>>()?;
                guarded(|| {
                    let path = CurvePath2::try_new(curves)?;
                    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)
                })
                .map(|r| r.boundary_loops().len())
            },
        );
    }

    let pair = |kind: &str, n: usize| {
        let f = Embed::identity().frame();
        (
            region_from(&closed_loop(kind, n, 0.0), &f),
            region_from(&closed_loop(kind, n, 0.5), &f),
        )
    };
    atlas.operation(
        "regions",
        "boolean union",
        "boundary segments per operand",
        "Regularized union of two prebuilt regions: an n-segment loop and the same loop rotated by half a step, whose boundaries cross 2n times.",
    );
    for kind in LOOP_KINDS {
        atlas.sweep(
            kind,
            segments,
            |n, _| pair(kind, n as usize),
            |(a, b)| {
                let (a, b) = (a.as_ref()?, b.as_ref()?);
                guarded(|| a.boolean_region(b, BooleanOp::Union)).map(|r| r.boundary_loops().len())
            },
        );
    }
    atlas.fuzz(
        "random star-shaped polygons",
        &powers_of_two(2, 7),
        |n, rng| {
            let f = Embed::identity().frame();
            let dx = (RADIUS * (rng.unit() - 0.5)) as i64;
            (
                region_from(&random_polygon(n as usize, rng, 0), &f),
                region_from(&random_polygon(n as usize, rng, dx), &f),
            )
        },
        |(a, b)| {
            let (a, b) = (a.as_ref()?, b.as_ref()?);
            guarded(|| a.boolean_region(b, BooleanOp::Union)).map(|r| r.boundary_loops().len())
        },
    );
    atlas.grid(
        "lines",
        ("first operand segments", &powers_of_two(2, 8)),
        ("second operand segments", &powers_of_two(2, 8)),
        |x, y, _| {
            let f = Embed::identity().frame();
            (
                region_from(&closed_loop("lines", x as usize, 0.0), &f),
                region_from(&closed_loop("lines", y as usize, 0.37), &f),
            )
        },
        |(a, b)| {
            let (a, b) = (a.as_ref()?, b.as_ref()?);
            guarded(|| a.boolean_region(b, BooleanOp::Union)).map(|r| r.boundary_loops().len())
        },
    );

    atlas.operation(
        "regions",
        "boolean all four (batched)",
        "boundary segments per operand",
        "Union, intersection, difference and xor of the same rotated pair from one shared evaluation.",
    );
    for kind in ["lines", "arcs", "cubic Béziers"] {
        atlas.sweep(
            kind,
            segments,
            |n, _| pair(kind, n as usize),
            |(a, b)| {
                let (a, b) = (a.as_ref()?, b.as_ref()?);
                guarded(|| a.boolean_regions(b)).map(|_| ())
            },
        );
    }

    atlas.operation(
        "regions",
        "classify_point",
        "boundary segments",
        "Classify a random point against a prebuilt n-segment region (warm: query indexes are reused).",
    );
    for kind in LOOP_KINDS {
        atlas.sweep(
            kind,
            segments,
            |n, rng| {
                let region = region_from(&closed_loop(kind, n as usize, 0.0), &identity.frame());
                let r = RADIUS as i64;
                let point = Point2::new(real(&int(rng.range(-r, r))), real(&int(rng.range(-r, r))));
                (region, CurvePoint2::from(point))
            },
            |(region, point)| region.as_ref()?.classify_point(point).ok().map(|_| ()),
        );
    }

    atlas.operation(
        "regions",
        "straight skeleton",
        "polygon edges",
        "Staged exact straight skeleton of a line contour (cold). Vertex trajectories and global contact events of an orthogonal comb (many reflex vertices), and the full skeleton of the comb and of a rounded regular polygon. A blocked stage is an error: on this revision the comb's global contact events block on an undecided predicate, its full skeleton with InvalidSplitTopology, and near-regular polygons (n ≥ 16) with UncertainWavefrontRelation.",
    );
    let comb = |teeth: i64| -> Vec<P> {
        let width = 4 * teeth + 2;
        let height = 4 * teeth + 4;
        let mut points = vec![pt(0, 0), pt(width, 0), pt(width, height)];
        for tooth in (0..teeth).rev() {
            let right = 4 * tooth + 4;
            let left = 4 * tooth + 2;
            let depth = tooth + 1;
            points.extend([
                pt(right, height),
                pt(right, depth),
                pt(left, depth),
                pt(left, height),
            ]);
        }
        points.push(pt(0, height));
        points
    };
    fn contour(points: &[P]) -> Option<Contour2> {
        let f = Embed::identity().frame();
        let n = points.len();
        let segments = (0..n)
            .map(|i| Shape::Line(points[i].clone(), points[(i + 1) % n].clone()).segment(&f))
            .collect::<Option<Vec<_>>>()?;
        Contour2::try_new(segments).ok()
    }
    let full = |points: &Vec<P>| {
        let contour = contour(points)?;
        guarded(|| contour.straight_skeleton())
            .filter(|r| r.skeleton().is_some())
            .map(|r| r.event_count())
    };
    let edges = powers_of_two(3, 9);
    atlas.sweep(
        "comb: vertex trajectories",
        &edges,
        |n, _| comb(((n as i64) - 4) / 4),
        |points| {
            let contour = contour(points)?;
            guarded(|| contour.straight_skeleton_vertex_trajectories()).map(|t| t.len())
        },
    );
    atlas.sweep(
        "comb: global contact events",
        &edges,
        |n, _| comb(((n as i64) - 4) / 4),
        |points| {
            let contour = contour(points)?;
            guarded(|| contour.straight_skeleton_global_contact_events()).map(|t| t.len())
        },
    );
    atlas.sweep(
        "comb: full skeleton",
        &edges,
        |n, _| comb(((n as i64) - 4) / 4),
        full,
    );
    atlas.sweep(
        "near-regular polygon: full skeleton",
        &edges,
        |n, _| polygon(n as usize, 0.0, RADIUS),
        full,
    );

    atlas.operation(
        "regions",
        "marching_squares",
        "grid samples per axis",
        "Exact marching-squares region from an n×n grid of a disk field (prebuilt samples): integer samples, and rational samples on a 1/3-scaled grid.",
    );
    for (kind, scale) in [
        ("disk, integer samples", int(1)),
        ("disk, rational samples", ratio(1, 3)),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 9),
            |n, _| {
                // Odd integers symmetric about 0; the disk of radius n-1 keeps
                // every boundary sample (|coordinate| = n-1, corner farther) outside.
                let n = n as i64;
                let at = |i: i64| &int(2 * i - (n - 1)) * &scale;
                let axis: Vec<Real> = (0..n).map(|i| real(&at(i))).collect();
                let r = &int(n - 2) * &scale;
                let mut values = Vec::new();
                for y in 0..n {
                    for x in 0..n {
                        let (px, py) = (at(x), at(y));
                        values.push(real(&(&(&(&px * &px) + &(&py * &py)) - &(&r * &r))));
                    }
                }
                (axis.clone(), axis, values)
            },
            |(x, y, values)| {
                marching_squares(&MarchingSquaresGrid { x, y, values })
                    .ok()
                    .map(|r| r.boundary_loops().len())
            },
        );
    }
}

fn weights(atlas: &mut Atlas) {
    let exponents: Vec<f64> = (-32..=32).step_by(4).map(f64::from).collect();
    let weighted = |kind: &str, k: i32| -> Shape {
        let w = pow2(k);
        match kind {
            "conic (1, w, 1)" => Shape::Conic([pt(-2, 0), pt(0, 2), pt(2, 0)], [int(1), w, int(1)]),
            "rational cubic (1, w, w, 1)" => Shape::RationalBezier(
                vec![pt(-3, 0), pt(-1, 3), pt(1, 3), pt(3, 0)],
                vec![int(1), w.clone(), w, int(1)],
            ),
            _ => Shape::Nurbs(
                3,
                (0..7)
                    .map(|i| pt(i - 3, if i % 2 == 0 { 1 } else { 3 }))
                    .collect(),
                (0..7)
                    .map(|i| if i % 2 == 0 { int(1) } else { w.clone() })
                    .collect(),
                clamped(3, 4),
            ),
        }
    };
    const WEIGHT_KINDS: [&str; 3] = [
        "conic (1, w, 1)",
        "rational cubic (1, w, w, 1)",
        "NURBS (alternating 1, w)",
    ];
    let make = |kind: &str, k: i32, rng: &mut Rng| {
        let curve = weighted(kind, k).curve(&Embed::identity().frame()).unwrap();
        (curve, unit_parameter(rng))
    };

    atlas.operation(
        "rational weights",
        "point_at by weight",
        "weight exponent k (w = 2^k)",
        "Evaluate a prebuilt rational curve (warm) whose interior weights are 2^k, approximated to f64; w → 0 hugs the chord, w → ∞ the control polygon.",
    );
    for kind in WEIGHT_KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |k, rng| make(kind, k as i32, rng),
            |(curve, t)| curve.point_at(&parameter(t)).ok().and_then(|p| force(&p)),
        );
    }

    atlas.operation(
        "rational weights",
        "bounds by weight",
        "weight exponent k (w = 2^k)",
        "Rebuild a rational curve with interior weights 2^k and compute its enclosure from the actual denominator (cold).",
    );
    for kind in WEIGHT_KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| weighted(kind, k as i32),
            |shape| {
                let curve = shape.curve(&Embed::identity().frame())?;
                let b = curve.bounds().ok()?;
                Some(b.min_y().to_f64_lossy()? + b.max_y().to_f64_lossy()?)
            },
        );
    }

    atlas.operation(
        "rational weights",
        "line intersection by weight",
        "weight exponent k (w = 2^k)",
        "Exact contacts of a rational curve with interior weights 2^k and the vertical line x = 1/3 (cold).",
    );
    let vertical = Shape::Line((ratio(1, 3), int(-1)), (ratio(1, 3), int(4)));
    for kind in WEIGHT_KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| weighted(kind, k as i32),
            |shape| intersect(&vertical, shape, &Embed::identity()),
        );
    }

    atlas.operation(
        "rational weights",
        "point_locations by weight",
        "weight exponent k (w = 2^k)",
        "Locate an on-curve point of a prebuilt rational curve with interior weights 2^k (warm). An empty answer is an error.",
    );
    for kind in WEIGHT_KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |k, rng| {
                let (curve, t) = make(kind, k as i32, rng);
                let point = on_curve(&curve, &t).unwrap();
                (curve, point)
            },
            |(curve, point)| locations(curve, point).filter(|&count| count > 0),
        );
    }
}

fn splines(atlas: &mut Atlas, degrees: &[f64]) {
    atlas.operation(
        "splines",
        "insert_knot by degree",
        "curve degree",
        "Rebuild a 4-span spline of growing degree (16-bit rational coordinates) and insert one 30-bit rational knot exactly (cold).",
    );
    for kind in ["polynomial B-spline", "NURBS"] {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| {
                let d = d as usize;
                let count = d + 4;
                let shape = if kind == "NURBS" {
                    degree_shape("NURBS", d)
                } else {
                    Shape::Spline(d, zigzag_points(count), clamped(d, 4))
                };
                (shape, Embed::rational(16, rng), unit_parameter(rng))
            },
            |(shape, embed, knot)| match shape.geometry(&embed.frame())? {
                CurveGeometry2::Nurbs(n) => n.insert_knot(real(knot)).ok().map(|n| n.degree()),
                CurveGeometry2::PolynomialBSpline(s) => {
                    s.insert_knot(real(knot)).ok().map(|s| s.degree())
                }
                _ => None,
            },
        );
    }

    atlas.operation(
        "splines",
        "elevate degree",
        "curve degree",
        "Rebuild a NURBS of degree d (16-bit rational coordinates) and elevate it exactly to degree d+1 (cold).",
    );
    for (kind, spans) in [("NURBS, 1 span", 1usize), ("NURBS, 4 spans", 4)] {
        atlas.sweep(
            kind,
            degrees,
            |d, rng| {
                let d = d as usize;
                let count = d + spans;
                let shape = Shape::Nurbs(
                    d,
                    zigzag_points(count),
                    (0..count).map(|i| int(1 + (i % 2) as i64)).collect(),
                    clamped(d, spans),
                );
                (shape, Embed::rational(16, rng), d)
            },
            |(shape, embed, d)| {
                let nurbs = shape.nurbs(&embed.frame())?;
                nurbs.elevated_to_degree(d + 1).ok().map(|n| n.degree())
            },
        );
    }

    atlas.operation(
        "splines",
        "global interpolation",
        "data points",
        "Exact uniform cubic NURBS interpolation through n data points on a 16-bit placement (solves the exact collocation system).",
    );
    for (kind, num) in NUMS {
        atlas.sweep(
            kind,
            &powers_of_two(2, 8),
            |n, rng| {
                let data: Vec<P> = (0..n as i64).map(|i| pt(i, rng.range(-4, 4))).collect();
                (data, Embed::random(num, 16, rng))
            },
            |(data, embed)| {
                let points = embed.frame().points(data);
                guarded(|| NurbsCurve2::interpolate_uniform(3, points)).map(|n| n.degree())
            },
        );
    }

    atlas.operation(
        "splines",
        "Bézier extraction",
        "spans",
        "Rebuild a cubic spline with n spans and extract every span as an exact native Bézier (cold).",
    );
    for kind in ["polynomial B-spline", "NURBS"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 9),
            |s, rng| {
                let s = s as usize;
                let shape = if kind == "NURBS" {
                    nurbs_zigzag(s, 1)
                } else {
                    Shape::Spline(3, zigzag_points(s + 3), clamped(3, s))
                };
                (shape, Embed::rational(16, rng))
            },
            |(shape, embed)| {
                let curve = shape.curve(&embed.frame())?;
                curve.native_bezier_fragments().ok().map(|f| f.len())
            },
        );
    }
}

fn projection(atlas: &mut Atlas) {
    let tolerances: Vec<f64> = (0..=40).step_by(2).map(f64::from).collect();
    atlas.operation(
        "flattening",
        "flatten_certified",
        "tolerance exponent k (max error 2^-k)",
        "Certified Bézier-to-polyline flattening with an exact error budget 2^-k (depth limit 64), 16-bit rational coordinates (cold); segment count grows like 2^(k/2).",
    );
    for kind in ["quadratic Bézier", "cubic Bézier"] {
        atlas.sweep(
            kind,
            &tolerances,
            |k, rng| (pow2(-(k as i32)), Embed::rational(16, rng)),
            |(tolerance, embed)| {
                let options = BezierFlatteningOptions::try_new(real(tolerance), 64).ok()?;
                let f = embed.frame();
                if kind.starts_with("quadratic") {
                    QuadraticBezier2::new(
                        f.point(&pt(-2, 4)),
                        f.point(&pt(0, -4)),
                        f.point(&pt(2, 4)),
                    )
                    .flatten_certified(&options)
                    .ok()
                    .map(|p| p.points().len())
                } else {
                    let Shape::Cubic([a, b, c, d]) = s_cubic() else {
                        return None;
                    };
                    CubicBezier2::new(f.point(&a), f.point(&b), f.point(&c), f.point(&d))
                        .flatten_certified(&options)
                        .ok()
                        .map(|p| p.points().len())
                }
            },
        );
    }

    atlas.operation(
        "flattening",
        "project_to_finite_polyline",
        "tolerance exponent k (chord error 2^-k)",
        "Lossy finite f64 polyline of a rebuilt one-curve path at chord error 2^-k (cold).",
    );
    for kind in FAMILIES {
        let shape = family_shape(kind);
        atlas.sweep(
            kind,
            &tolerances,
            |k, rng| (2f64.powi(-(k as i32)), Embed::rational(16, rng)),
            |(tolerance, embed)| {
                let options = FiniteProjectionOptions::try_new(*tolerance).ok()?;
                let path = path_from(std::slice::from_ref(&shape), &embed.frame())?;
                path.project_to_finite_polyline(&options)
                    .ok()
                    .map(|p| p.points().len())
            },
        );
    }
}

#[cfg(feature = "svg")]
fn svg(atlas: &mut Atlas, segments: &[f64]) {
    use hypercurve::{
        SvgOptions, export_svg_document_with_options, import_svg_document, parse_svg_path_data,
    };
    use std::fmt::Write as _;

    /// Path data of n commands of one kind along a zigzag, with coordinates
    /// of `digits` significant decimal digits when nonzero.
    fn path_data(kind: char, n: usize, digits: u32, rng: &mut Rng) -> String {
        let mut number = |value: f64| -> String {
            if digits == 0 {
                format!("{value}")
            } else {
                let mantissa: String = (0..digits)
                    .map(|_| char::from(b'0' + rng.below(10) as u8))
                    .collect();
                format!("{}.{mantissa}", value as i64)
            }
        };
        let mut d = String::from("M0 0");
        for i in 0..n {
            let x = i as f64 + 1.0;
            let y = if i % 2 == 0 { 1.0 } else { 0.0 };
            let _ = match kind {
                'L' => write!(d, " L{} {}", number(x), number(y)),
                'Q' => write!(
                    d,
                    " Q{} {} {} {}",
                    number(x - 0.5),
                    number(2.0 * y - 0.5),
                    number(x),
                    number(y)
                ),
                'C' => write!(
                    d,
                    " C{} {} {} {} {} {}",
                    number(x - 0.75),
                    number(y + 1.0),
                    number(x - 0.25),
                    number(y - 1.0),
                    number(x),
                    number(y)
                ),
                _ => write!(d, " A1 1 0 0 1 {} {}", number(x), number(y)),
            };
        }
        d
    }

    /// A closed regular-polygon-based path of n commands of one kind. `A`
    /// arcs have radius R (nearly co-circular with the vertex circle), `B`
    /// arcs radius 2R; both have irrational centers after SVG's endpoint
    /// parameterization.
    fn closed_document(kind: char, n: usize) -> String {
        let r = 1_000_000.0;
        let vertex = |i: usize| {
            let theta = 2.0 * PI * i as f64 / n as f64;
            ((r * theta.cos()).round(), (r * theta.sin()).round())
        };
        let (x0, y0) = vertex(0);
        let mut d = format!("M{x0} {y0}");
        let h = 4.0 / 3.0 * (PI / n as f64 / 2.0).tan() * r;
        for i in 1..=n {
            let (x, y) = vertex(i % n);
            let _ = match kind {
                'L' => write!(d, " L{x} {y}"),
                'C' => {
                    let ta = 2.0 * PI * (i - 1) as f64 / n as f64;
                    let tb = 2.0 * PI * i as f64 / n as f64;
                    write!(
                        d,
                        " C{} {} {} {} {x} {y}",
                        (r * ta.cos() - h * ta.sin()).round(),
                        (r * ta.sin() + h * ta.cos()).round(),
                        (r * tb.cos() + h * tb.sin()).round(),
                        (r * tb.sin() - h * tb.cos()).round()
                    )
                }
                'A' => write!(d, " A{r} {r} 0 0 1 {x} {y}"),
                _ => write!(d, " A{} {} 0 0 1 {x} {y}", 2.0 * r, 2.0 * r),
            };
        }
        d.push_str(" Z");
        format!(r#"<svg xmlns="http://www.w3.org/2000/svg"><path d="{d}"/></svg>"#)
    }

    atlas.operation(
        "svg",
        "parse_svg_path_data by commands",
        "path commands",
        "Parse SVG path data of n commands into exact native subpaths (decimal coordinates become exact rationals).",
    );
    for (kind, command) in [
        ("L (lines)", 'L'),
        ("Q (quadratics)", 'Q'),
        ("C (cubics)", 'C'),
        ("A (arcs)", 'A'),
    ] {
        atlas.sweep(
            kind,
            segments,
            |n, rng| path_data(command, n as usize, 0, rng),
            |d| parse_svg_path_data(d).ok().map(|s| s.len()),
        );
    }
    atlas.fuzz(
        "random commands and digits",
        &powers_of_two(2, 8),
        |n, rng| {
            let command = ['L', 'Q', 'C', 'A'][rng.below(4) as usize];
            let digits = rng.below(24) as u32;
            path_data(command, n as usize, digits, rng)
        },
        |d| parse_svg_path_data(d).ok().map(|s| s.len()),
    );

    atlas.operation(
        "svg",
        "parse_svg_path_data by digits",
        "decimal digits per coordinate",
        "Parse 16 path commands whose coordinates carry growing numbers of decimal fraction digits.",
    );
    for (kind, command) in [("L (lines)", 'L'), ("C (cubics)", 'C')] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |digits, rng| path_data(command, 16, digits as u32, rng),
            |d| parse_svg_path_data(d).ok().map(|s| s.len()),
        );
    }

    const SVG_LOOPS: [(&str, char); 4] = [
        ("polygon (L)", 'L'),
        ("cubic loop (C)", 'C'),
        ("arc polygon, radius 2R (A)", 'B'),
        ("near-co-circular arcs, radius R (A)", 'A'),
    ];
    atlas.operation(
        "svg",
        "import_svg_document",
        "path commands",
        "Import a filled SVG document with one closed path of n commands, including exact arrangement into a CurveRegion2.",
    );
    for (kind, command) in SVG_LOOPS {
        atlas.sweep(
            kind,
            segments,
            |n, _| closed_document(command, n as usize),
            |doc| guarded(|| import_svg_document(doc)).map(|g| g.region().boundary_loops().len()),
        );
    }

    atlas.operation(
        "svg",
        "export_svg_document",
        "path commands",
        "Export a prebuilt imported region (exact data-hypercurve-path plus finite display path) to an SVG document. The display tolerance is 100 units (1e-4 of the 1e6 radius): the default absolute 0.1 needs about 7000 segments per curved loop and fails with SizeOverflow on the 4096-segment cap.",
    );
    // Arc documents above 64 commands take seconds to import (untimed setup).
    let arc_segments = powers_of_two(2, 6);
    for (kind, command) in SVG_LOOPS {
        atlas.sweep(
            kind,
            if matches!(command, 'A' | 'B') {
                &arc_segments
            } else {
                segments
            },
            |n, _| import_svg_document(&closed_document(command, n as usize)).ok(),
            |geometry| {
                let options = SvgOptions {
                    curve_tolerance: 100.0,
                    ..SvgOptions::default()
                };
                export_svg_document_with_options(geometry.as_ref()?, options)
                    .ok()
                    .map(|s| s.len())
            },
        );
    }
}
