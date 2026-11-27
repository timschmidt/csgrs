//! Performance atlas for HyperBRep: exact B-rep construction, regularized
//! Booleans, local topology edits, solid queries, archive replay, exact
//! intersection predicates and derived tessellation.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperbrep --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
//! The tessellation group needs the `tessellation` feature (as
//! `--all-features` enables).
//!
//! Axes:
//! - profile vertices `n` of an extruded, revolved, swept or planar polygon
//!   (a convex `n`-gon prism has `n + 2` faces);
//! - operand sides `n` for Booleans of two crossing prisms, with `n` × `m`
//!   heatmaps, and the operand count of chained Booleans;
//! - coordinate bits of exact rational inputs;
//! - distance to degeneracy `2^-k` (near-coincident faces, near-tangent
//!   sections, near-parallel lines, points near a boundary);
//! - hole, void and loft-section counts; tessellation segments and levels.
//!
//! Input kinds: convex polygons with exactly cocircular rational vertices,
//! concave star polygons (alternating radii), axis-aligned boxes, the
//! analytic primitives (cylinder, sphere, cone frustum, torus), and left-fold
//! versus balanced Boolean trees.
//!
//! Timed queries on prebuilt solids receive a fresh copy (`transformed` by
//! the identity, which publishes a new revision with empty caches), so bounds
//! or face contours cached by an earlier call are never reused. Kinds
//! labelled "warm" deliberately reuse one revision.
//!
//! With `--resources`, results report their structure:
//! - solids and sheets: `nodes` = faces, `depth` = deepest exact
//!   vertex-coordinate expression tree, `bits` = largest stored rational in
//!   a vertex coordinate;
//! - chained Booleans: as solids, but `depth` = CSG nesting depth of the
//!   Boolean tree (the operand count for a left fold, about `log2` of it for
//!   a balanced tree);
//! - measures (volume, area): the exact scalar's expression nodes, depth and
//!   bits;
//! - tessellations: `nodes` = triangles; `depth` and `bits` as for vertices;
//! - intersection graphs: `nodes` = exactly intersecting face pairs.
//!
//! Not covered: fillets, chamfers, offsets, shelling and thickening have no
//! public operation yet (they are targets in `AGENTS.md`).
use hyperbrep::{
    ArchiveLimits, BodySet, BooleanOperation, Curve3, Direction, FaceRef, LoftSection, Matrix4,
    Model, Parameter, Point2, Point3, RawModel, Real, SheetSet, SolidIntersectionGraph, Surface,
    Vector3, boolean, builder,
};
use perf_atlas::{Atlas, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

// ---------------------------------------------------------------------------
// Exact scalars and points.

fn int(v: i64) -> Real {
    Real::from(v)
}

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).expect("nonzero denominator")
}

/// `2^-k`, exactly.
fn gap(k: u32) -> Real {
    let mut e = Real::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        e *= q(1, 1 << step);
        left -= step;
    }
    e
}

/// A random integer with exactly `bits` bits, built from 32-bit limbs.
fn big(rng: &mut Rng, bits: u32) -> Real {
    let mut value = Real::zero();
    for limb in rng.limbs(bits).iter().rev() {
        value = value * int(1 << 32) + int(i64::from(*limb));
    }
    value
}

/// A random exact fraction in `(0, 1/2)` whose parts have about `bits` bits.
fn fraction(rng: &mut Rng, bits: u32) -> Real {
    let numerator = big(rng, bits);
    let denominator = big(rng, bits + 1) * int(2) + int(1);
    (numerator / denominator).expect("odd denominator")
}

fn point(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn ipoint(x: i64, y: i64, z: i64) -> Point3 {
    point(int(x), int(y), int(z))
}

fn vector(x: i64, y: i64, z: i64) -> Vector3 {
    Vector3::new([int(x), int(y), int(z)])
}

/// Row-major affine map with linear part `linear` and translation `offset`.
fn affine(linear: [[Real; 3]; 3], offset: [Real; 3]) -> Matrix4 {
    let [r0, r1, r2] = linear;
    let [x, y, z] = offset;
    let [a, b, c] = r0;
    let [d, e, f] = r1;
    let [g, h, i] = r2;
    Matrix4::from_row_major([
        a,
        b,
        c,
        x,
        d,
        e,
        f,
        y,
        g,
        h,
        i,
        z,
        int(0),
        int(0),
        int(0),
        int(1),
    ])
}

fn translation(x: Real, y: Real, z: Real) -> Matrix4 {
    affine(
        [
            [int(1), int(0), int(0)],
            [int(0), int(1), int(0)],
            [int(0), int(0), int(1)],
        ],
        [x, y, z],
    )
}

/// Rigid rotation about z by the 3-4-5 angle, then a translation.
fn rotation() -> Matrix4 {
    affine(
        [
            [q(3, 5), q(-4, 5), int(0)],
            [q(4, 5), q(3, 5), int(0)],
            [int(0), int(0), int(1)],
        ],
        [int(1), int(2), int(3)],
    )
}

/// Exact rational point on the circle of radius `r` about `(cx, cy)` near
/// angle `theta`, from the rational parametrisation with `tan(theta / 2)`
/// rounded to a multiple of `2^-12`.
fn circle_point(theta: f64, r: &Real, cx: &Real, cy: &Real) -> Point2 {
    const S: i64 = 1 << 12;
    let t = ((theta / 2.0).tan() * S as f64).round() as i64;
    let d = S * S + t * t;
    let x = q(S * S - t * t, d);
    let y = q(2 * S * t, d);
    Point2::new(cx.clone() + r.clone() * x, cy.clone() + r.clone() * y)
}

/// An `n`-gon: convex with cocircular vertices, or a concave star whose odd
/// vertices lie on a circle of half the radius. Angles span (-2.9, 2.9) so
/// `tan(theta / 2)` stays bounded.
fn polygon(n: usize, star: bool, r: &Real, (cx, cy): (&Real, &Real), phase: f64) -> Vec<Point2> {
    (0..n)
        .map(|i| {
            let theta = -2.9 + 5.8 * (i as f64 + phase) / n as f64;
            let radius = if star && i % 2 == 1 {
                r.clone() * q(1, 2)
            } else {
                r.clone()
            };
            circle_point(theta, &radius, cx, cy)
        })
        .collect()
}

fn rectangle(x0: i64, y0: i64, x1: i64, y1: i64) -> Vec<Point2> {
    vec![
        Point2::new(int(x0), int(y0)),
        Point2::new(int(x1), int(y0)),
        Point2::new(int(x1), int(y1)),
        Point2::new(int(x0), int(y1)),
    ]
}

fn is_star(kind: &str) -> bool {
    kind.starts_with("star")
}

/// First Boolean operand: an `n`-gon prism of radius 4 about the origin, z in [0, 2].
fn left_prism(n: usize, star: bool) -> BodySet {
    let profile = polygon(n, star, &int(4), (&int(0), &int(0)), 0.0);
    builder::extrude(&profile, int(0), int(2)).expect("left prism")
}

/// Second operand: an `m`-gon prism about `offset`, z in [1, 3], so no face
/// is coplanar with the first and every side wall is cut.
fn right_prism(m: usize, star: bool, offset: (Real, Real), phase: f64) -> BodySet {
    let profile = polygon(m, star, &int(4), (&offset.0, &offset.1), phase);
    builder::extrude(&profile, int(1), int(3)).expect("right prism")
}

fn default_offset() -> (Real, Real) {
    (q(1, 3), q(1, 7))
}

fn crossing_prisms(n: usize, m: usize, star: bool) -> (BodySet, BodySet) {
    (
        left_prism(n, star),
        right_prism(m, star, default_offset(), 0.37),
    )
}

/// A fresh revision of `master` (new caches), built outside the timer.
fn fresh(master: &BodySet) -> BodySet {
    master
        .transformed(&Matrix4::identity())
        .expect("identity copy")
}

/// The master for magnitude `m`, rebuilt only when the magnitude changes.
fn memo<T>(slot: &mut Option<(f64, T)>, m: f64, build: impl FnOnce() -> T) -> &T {
    if slot.as_ref().is_none_or(|(key, _)| *key != m) {
        *slot = Some((m, build()));
    }
    &slot.as_ref().expect("memoised master").1
}

fn solid_of(bodies: &BodySet) -> hyperbrep::SolidRef<'_> {
    bodies.bodies().next().expect("one body")
}

/// The face whose outer wire has the most edge uses (a prism cap).
fn cap(model: &Model) -> FaceRef<'_> {
    model
        .faces()
        .max_by_key(|face| face.outer().map_or(0, |wire| wire.edge_uses().len()))
        .expect("a face")
}

// ---------------------------------------------------------------------------
// Shape readers (resource atlases only; public API only).

fn scalar_size(value: &Real) -> (u64, u64, u64) {
    let size = value.representation_size(1 << 20);
    (size.nodes as u64, size.depth as u64, size.max_rational_bits)
}

/// Deepest expression and largest stored rational among the coordinates.
fn coordinates<'a>(points: impl Iterator<Item = &'a Point3>) -> (u64, u64) {
    let mut depth = 0;
    let mut bits = 0;
    for p in points {
        for c in [&p.x, &p.y, &p.z] {
            let (_, d, b) = scalar_size(c);
            depth = depth.max(d);
            bits = bits.max(b);
        }
    }
    (depth, bits)
}

fn model_shape(model: &Model) -> Shape {
    let vertices: Vec<_> = model.vertices().collect();
    let (depth, bits) = coordinates(
        vertices
            .iter()
            .filter_map(|vertex| vertex.point().try_coordinates()),
    );
    Shape {
        nodes: model.counts().faces as u64,
        depth,
        bits,
    }
}

fn bodies_shape<E>(result: &Result<BodySet, E>) -> Option<Shape> {
    result
        .as_ref()
        .ok()
        .map(|bodies| model_shape(bodies.model()))
}

fn sheet_shape<E>(result: &Result<SheetSet, E>) -> Option<Shape> {
    result.as_ref().ok().map(|sheet| model_shape(sheet.model()))
}

fn revision_shape<E>(result: &Result<Model, E>) -> Option<Shape> {
    result.as_ref().ok().map(model_shape)
}

/// A Boolean tree result with its CSG nesting depth.
fn chain_shape<E>(result: &Result<(BodySet, u64), E>) -> Option<Shape> {
    result.as_ref().ok().map(|(bodies, depth)| Shape {
        depth: *depth,
        ..model_shape(bodies.model())
    })
}

fn real_shape<E>(result: &Result<Real, E>) -> Option<Shape> {
    result.as_ref().ok().map(|value| {
        let (nodes, depth, bits) = scalar_size(value);
        Shape { nodes, depth, bits }
    })
}

fn graph_shape<E>(result: &Result<SolidIntersectionGraph, E>) -> Option<Shape> {
    result.as_ref().ok().map(|graph| Shape {
        nodes: graph.exact_intersection_pairs() as u64,
        depth: 0,
        bits: 0,
    })
}

fn shaped_bodies<E>(result: Result<BodySet, E>) -> Shaped<Result<BodySet, E>> {
    Shaped::new(result, bodies_shape::<E>)
}

// ---------------------------------------------------------------------------
// Boolean trees.

/// Fold `operands` into `base` left to right (CSG depth `k`), or combine base
/// and operands pairwise as a balanced tree (depth `ceil(log2(k + 1))`).
fn tree(
    base: &BodySet,
    operands: &[BodySet],
    operation: BooleanOperation,
    balanced: bool,
) -> Result<(BodySet, u64), hyperbrep::BooleanError> {
    if !balanced {
        let mut acc = base.clone();
        for operand in operands {
            acc = boolean::boolean(&acc, operand, operation)?;
        }
        return Ok((acc, operands.len() as u64));
    }
    let mut level: Vec<BodySet> = std::iter::once(base.clone())
        .chain(operands.iter().cloned())
        .collect();
    let mut depth = 0;
    while level.len() > 1 {
        let mut next = Vec::with_capacity(level.len().div_ceil(2));
        for pair in level.chunks(2) {
            next.push(match pair {
                [a, b] => boolean::boolean(a, b, operation)?,
                [a] => a.clone(),
                _ => unreachable!("chunks of two"),
            });
        }
        level = next;
        depth += 1;
    }
    Ok((level.pop().expect("one result"), depth))
}

/// A staircase of `k + 1` overlapping boxes, each shifted by (1/2, 1/3) and
/// one third taller than the last.
fn staircase(k: usize) -> (BodySet, Vec<BodySet>) {
    let step = |i: i64| {
        builder::cuboid(
            point(q(i, 2), q(i, 3), int(0)),
            point(q(i, 2) + int(1), q(i, 3) + int(1), q(i + 3, 3)),
        )
        .expect("staircase box")
    };
    (step(0), (1..=k as i64).map(step).collect())
}

/// A plate with `k` cutters in a row: square pockets or through drills.
fn plate_with_cutters(k: usize, drills: bool) -> (BodySet, Vec<BodySet>) {
    let plate = builder::cuboid(Point3::origin(), ipoint(3 * k as i64 + 1, 3, 1)).expect("plate");
    let cutters = (0..k as i64)
        .map(|i| {
            if drills {
                builder::cylinder(q(1, 3), int(2))
                    .expect("drill")
                    .transformed(&translation(q(6 * i + 3, 2), q(3, 2), q(-1, 2)))
                    .expect("placed drill")
            } else {
                builder::cuboid(
                    point(int(3 * i + 1), int(1), q(1, 2)),
                    ipoint(3 * i + 2, 2, 2),
                )
                .expect("pocket")
            }
        })
        .collect();
    (plate, cutters)
}

// ---------------------------------------------------------------------------
// Analytic primitives.

const PRIMITIVES: [&str; 4] = ["cylinder", "sphere", "cone frustum", "torus"];

fn primitive(kind: &str, scale: &Real) -> Result<BodySet, hyperbrep::ConstructionError> {
    let s = || scale.clone();
    match kind {
        "cylinder" => builder::cylinder(s(), int(2) * s()),
        "sphere" => builder::sphere(s()),
        "cone frustum" => builder::cone_frustum(int(2) * s(), s(), int(2) * s()),
        "torus" => builder::torus(int(3) * s(), s()),
        _ => unreachable!("unknown primitive {kind}"),
    }
}

fn unit_primitive(kind: &str) -> BodySet {
    primitive(kind, &int(1)).expect("unit primitive")
}

/// Lowest cut height that still crosses a unit primitive's material.
fn cut_base(kind: &str) -> Real {
    match kind {
        "sphere" | "torus" => q(1, 4),
        _ => q(3, 4),
    }
}

/// A solid and a point at distance `e` inside its boundary.
fn near_boundary(kind: &str, e: Real) -> (BodySet, Point3) {
    match kind {
        "planar face" => (
            builder::cuboid(Point3::origin(), ipoint(2, 2, 2)).expect("box"),
            point(int(2) - e, int(1), int(1)),
        ),
        "cylinder" => (
            unit_primitive("cylinder"),
            point(int(1) - e, int(0), int(1)),
        ),
        "sphere" => (unit_primitive("sphere"), point(int(1) - e, int(0), int(0))),
        "torus" => (unit_primitive("torus"), point(int(4) - e, int(0), int(0))),
        _ => unreachable!("unknown boundary kind {kind}"),
    }
}

// ---------------------------------------------------------------------------
// Carrier predicates.

fn plane_z(z: Real) -> Surface {
    Surface::plane(point(int(0), int(0), z), vector(1, 0, 0), vector(0, 1, 0)).expect("plane")
}

fn unit_surface(kind: &str) -> Surface {
    let (x, y, z) = (vector(1, 0, 0), vector(0, 1, 0), vector(0, 0, 1));
    match kind {
        "sphere" => Surface::sphere(Point3::origin(), x, y, z, int(1)),
        "cylinder" => Surface::cylinder(Point3::origin(), x, y, z, int(1)),
        "torus" => Surface::torus(Point3::origin(), x, y, z, int(3), int(1)),
        _ => unreachable!("unknown surface {kind}"),
    }
    .expect("surface")
}

/// Two carriers whose section degenerates to tangency (or, for two planes,
/// to parallelism) as `k` grows.
fn near_tangent_surfaces(kind: &str, k: u32) -> (Surface, Surface) {
    let e = gap(k);
    match kind {
        "plane–sphere" => (plane_z(int(1) - e), unit_surface("sphere")),
        "plane–torus" => (plane_z(int(1) - e), unit_surface("torus")),
        "plane–cylinder" => (
            Surface::plane(
                point(int(1) - e, int(0), int(0)),
                vector(0, 1, 0),
                vector(0, 0, 1),
            )
            .expect("plane"),
            unit_surface("cylinder"),
        ),
        "plane–plane" => (
            plane_z(int(0)),
            Surface::plane(
                Point3::origin(),
                Vector3::new([int(1), int(0), e]),
                vector(0, 1, 0),
            )
            .expect("tilted plane"),
        ),
        _ => unreachable!("unknown section {kind}"),
    }
}

fn line(a: Point3, b: Point3) -> Curve3 {
    Curve3::line(a, b).expect("line")
}

/// A line `2^-k` inside tangency with a unit carrier.
fn near_tangent_line(kind: &str, k: u32) -> (Curve3, Surface) {
    let h = int(1) - gap(k);
    match kind {
        "line–sphere" => (
            line(point(int(-3), int(0), h.clone()), point(int(3), int(0), h)),
            unit_surface("sphere"),
        ),
        "line–torus" => (
            line(point(int(-5), int(0), h.clone()), point(int(5), int(0), h)),
            unit_surface("torus"),
        ),
        "line–cylinder" => (
            line(point(h.clone(), int(-3), int(1)), point(h, int(3), int(1))),
            unit_surface("cylinder"),
        ),
        _ => unreachable!("unknown contact {kind}"),
    }
}

/// Two segments whose directions differ by about `2^-k`.
fn near_parallel_lines(kind: &str, k: u32) -> (Curve3, Curve3) {
    let e = gap(k);
    let first = line(Point3::origin(), ipoint(4, 0, 0));
    let second = match kind {
        "crossing" => line(point(int(0), -e.clone(), int(0)), point(int(4), e, int(0))),
        "skew" => line(point(int(0), -e.clone(), int(1)), point(int(4), e, int(1))),
        "parallel offset" => line(point(int(0), e.clone(), int(0)), point(int(4), e, int(0))),
        _ => unreachable!("unknown line pair {kind}"),
    };
    (first, second)
}

// ---------------------------------------------------------------------------
// Planar sketches.

fn curve_path(profile: &[Point2]) -> hypercurve::CurvePath2 {
    let points: Vec<_> = profile
        .iter()
        .map(|p| hypercurve::Point2::new(p.x.clone(), p.y.clone()))
        .collect();
    let segments = (0..points.len())
        .map(|i| {
            hypercurve::Curve2::from(
                hypercurve::LineSeg2::try_new(
                    points[i].clone(),
                    points[(i + 1) % points.len()].clone(),
                )
                .expect("segment"),
            )
        })
        .collect();
    hypercurve::CurvePath2::try_new(segments).expect("closed path")
}

fn sketch(n: usize, star: bool) -> Result<SheetSet, hyperbrep::ConstructionError> {
    let profile = polygon(n, star, &int(4), (&int(0), &int(0)), 0.0);
    builder::planar_face(
        &curve_path(&profile),
        &[],
        Point3::origin(),
        vector(1, 0, 0),
        vector(0, 1, 0),
    )
}

fn main() {
    let mut atlas = Atlas::from_args("hyperbrep");
    let vertices = powers_of_two(2, 10);
    let query_vertices = powers_of_two(2, 8);
    let bits = powers_of_two(0, 10);
    let gaps: Vec<f64> = (0..=8).map(|i| f64::from(i * 16)).collect();

    construction(&mut atlas, &vertices, &bits);
    booleans(&mut atlas, &bits, &gaps);
    edits(&mut atlas, &query_vertices);
    queries(&mut atlas, &query_vertices, &gaps);
    predicates(&mut atlas, &gaps);
    #[cfg(feature = "tessellation")]
    tessellate::run(&mut atlas, &vertices);
    atlas.finish();
}

fn construction(atlas: &mut Atlas, vertices: &[f64], bits: &[f64]) {
    atlas.operation(
        "construction",
        "cuboid",
        "coordinate bits",
        "Axis-aligned box with exact corners of the given size: integers, dyadics (b-bit numerator over 2^b) or general rationals.",
    );
    for kind in ["integer", "dyadic", "rational"] {
        atlas.sweep(
            kind,
            bits,
            |b, rng| {
                let b = b as u32;
                let coordinate = |rng: &mut Rng| match kind {
                    "integer" => big(rng, b),
                    "dyadic" => big(rng, b) * gap(b),
                    _ => fraction(rng, b),
                };
                let min = point(coordinate(rng), coordinate(rng), coordinate(rng));
                let max = point(
                    min.x.clone() + int(1) + coordinate(rng),
                    min.y.clone() + int(1) + coordinate(rng),
                    min.z.clone() + int(1) + coordinate(rng),
                );
                (min, max)
            },
            |(min, max)| shaped_bodies(builder::cuboid(min.clone(), max.clone())),
        );
    }

    atlas.operation(
        "construction",
        "extrude polygon",
        "profile vertices",
        "Prism from an exact simple polygon; certifies simplicity and publishes a validated solid.",
    );
    for kind in ["convex", "star (concave)"] {
        atlas.sweep(
            kind,
            vertices,
            |n, _| polygon(n as usize, is_star(kind), &int(4), (&int(0), &int(0)), 0.0),
            |profile| shaped_bodies(builder::extrude(profile, int(0), int(2))),
        );
    }

    atlas.operation(
        "construction",
        "extrude region with holes",
        "hole count",
        "A 3 × (3k + 1) plate with k unit-square through-holes in a row (one genus-k shell).",
    );
    atlas.sweep(
        "square holes",
        &powers_of_two(0, 7),
        |k, _| {
            let k = k as i64;
            let holes: Vec<_> = (0..k)
                .map(|i| rectangle(3 * i + 1, 1, 3 * i + 2, 2))
                .collect();
            (rectangle(0, 0, 3 * k + 1, 3), holes)
        },
        |(outer, holes)| shaped_bodies(builder::extrude_region(outer, holes, int(0), int(1))),
    );

    atlas.operation(
        "construction",
        "extrude with voids",
        "void count",
        "A plate with k strictly interior prismatic cavities; certifies pairwise non-contact of the void shells.",
    );
    atlas.sweep(
        "square voids",
        &powers_of_two(0, 6),
        |k, _| {
            let k = k as i64;
            let voids: Vec<_> = (0..k)
                .map(|i| builder::ExtrusionVoid {
                    profile: rectangle(3 * i + 1, 1, 3 * i + 2, 2),
                    z_min: q(1, 4),
                    z_max: q(3, 4),
                })
                .collect();
            (rectangle(0, 0, 3 * k + 1, 3), voids)
        },
        |(outer, voids)| shaped_bodies(builder::extrude_with_voids(outer, int(0), int(1), voids)),
    );

    atlas.operation(
        "construction",
        "revolve polygon",
        "profile vertices",
        "Full-turn revolution of an exact radial/axial polygon (radius in [2, 4]) about z; each segment yields four angular faces.",
    );
    for kind in ["convex", "star (concave)"] {
        atlas.sweep(
            kind,
            vertices,
            |n, _| polygon(n as usize, is_star(kind), &int(1), (&int(3), &int(0)), 0.0),
            |profile| shaped_bodies(builder::revolve(profile)),
        );
    }

    atlas.operation(
        "construction",
        "linear sweep",
        "profile vertices",
        "Oblique linear sweep of a polygon along (1, 1, 3) in an exact affine frame (a sheared prism).",
    );
    for kind in ["convex", "star (concave)"] {
        atlas.sweep(
            kind,
            vertices,
            |n, _| polygon(n as usize, is_star(kind), &int(1), (&int(0), &int(0)), 0.0),
            |profile| {
                shaped_bodies(builder::sweep(
                    profile,
                    Point3::origin(),
                    vector(1, 0, 0),
                    vector(0, 1, 0),
                    vector(1, 1, 3),
                ))
            },
        );
    }

    atlas.operation(
        "construction",
        "loft",
        "section count",
        "Loft through k + 1 corresponding octagons: homothetic (planar ruled sides) or twisted (bilinear rational Bézier sides with an exact convexity certificate).",
    );
    for kind in ["homothetic", "twisted"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 6),
            |k, _| {
                (0..=k as i64)
                    .map(|i| LoftSection {
                        profile: polygon(
                            8,
                            false,
                            &q(8 + (i % 3), 8),
                            (&int(0), &int(0)),
                            if kind == "twisted" {
                                0.1 * (i % 2) as f64
                            } else {
                                0.0
                            },
                        ),
                        z: int(i),
                    })
                    .collect::<Vec<_>>()
            },
            |sections| shaped_bodies(builder::loft(sections)),
        );
    }

    atlas.operation(
        "construction",
        "planar face",
        "profile vertices",
        "Trimmed planar sheet face from an exact closed line path (sketch to face); Hypercurve certifies path simplicity.",
    );
    for kind in ["convex", "star (concave)"] {
        atlas.sweep(
            kind,
            vertices,
            |n, _| n as usize,
            |&n| Shaped::new(sketch(n, is_star(kind)), sheet_shape),
        );
    }

    atlas.operation(
        "construction",
        "analytic primitive",
        "scale bits",
        "Exact cylinder, sphere, cone frustum and torus whose scale is 1 + f, with f a random rational of the given size.",
    );
    for kind in PRIMITIVES {
        atlas.sweep(
            kind,
            bits,
            |b, rng| int(1) + fraction(rng, b as u32),
            |scale| shaped_bodies(primitive(kind, scale)),
        );
    }

    atlas.operation(
        "construction",
        "sphere with voids",
        "void count",
        "A sphere of radius 4k with k disjoint unit spherical cavities on a diameter; certifies non-contact and containment.",
    );
    atlas.sweep(
        "spherical voids",
        &powers_of_two(0, 6),
        |k, _| {
            let k = k as i64;
            (0..k)
                .map(|i| builder::SphereVoid {
                    center: ipoint(4 * i - 2 * (k - 1), 0, 0),
                    radius: int(1),
                })
                .collect::<Vec<_>>()
        },
        |voids| {
            shaped_bodies(builder::sphere_with_voids(
                int(4 * voids.len() as i64),
                voids,
            ))
        },
    );
}

const BOOLEAN_OPS: [(&str, BooleanOperation); 3] = [
    ("union", BooleanOperation::Union),
    ("intersection", BooleanOperation::Intersection),
    ("difference", BooleanOperation::Difference),
];

fn booleans(atlas: &mut Atlas, bits: &[f64], gaps: &[f64]) {
    let sides = powers_of_two(2, 7);
    let grid_sides = powers_of_two(2, 6);
    let operands = powers_of_two(0, 6);
    for (name, operation) in BOOLEAN_OPS {
        atlas.operation(
            "boolean",
            &format!("prism {name}"),
            "operand sides (n for both)",
            &format!(
                "Regularized {name} of an n-gon prism and an n-gon prism offset by (1/3, 1/7) and by 1 in z: every side wall crosses and no faces coincide. Heatmap: n × m sides."
            ),
        );
        for kind in ["convex", "star (concave)"] {
            atlas.sweep(
                kind,
                &sides,
                |n, _| crossing_prisms(n as usize, n as usize, is_star(kind)),
                |(a, b)| shaped_bodies(boolean::boolean(a, b, operation)),
            );
        }
        if operation == BooleanOperation::Intersection {
            atlas.fuzz(
                "convex, random placement",
                &grid_sides[..4],
                |n, rng| {
                    let n = n as usize;
                    let offset = (
                        fraction(rng, 10) * int(4) - int(1),
                        fraction(rng, 10) * int(4) - int(1),
                    );
                    (
                        left_prism(n, false),
                        right_prism(n, false, offset, rng.unit()),
                    )
                },
                |(a, b)| shaped_bodies(boolean::boolean(a, b, operation)),
            );
        }
        atlas.grid(
            "convex",
            ("first operand sides", &grid_sides),
            ("second operand sides", &grid_sides),
            |n, m, _| crossing_prisms(n as usize, m as usize, false),
            |(a, b)| shaped_bodies(boolean::boolean(a, b, operation)),
        );
    }

    atlas.operation(
        "boolean",
        "prism symmetric difference",
        "operand sides (n for both)",
        "Regularized XOR of two crossing convex n-gon prisms: two directed differences and their union.",
    );
    atlas.sweep(
        "convex",
        &sides[..4],
        |n, _| crossing_prisms(n as usize, n as usize, false),
        |(a, b)| shaped_bodies(boolean::symmetric_difference(a, b)),
    );

    atlas.operation(
        "boolean",
        "box boolean by coordinate bits",
        "coordinate bits",
        "Boxes [0,2]^3 and [1+f, 3]^3 with random rational offsets f of the given size; coplanar kinds share the z = 0 and z = 2 planes.",
    );
    for kind in [
        "generic union",
        "generic intersection",
        "coplanar union",
        "coplanar intersection",
    ] {
        let operation = if kind.ends_with("union") {
            BooleanOperation::Union
        } else {
            BooleanOperation::Intersection
        };
        let coplanar = kind.starts_with("coplanar");
        atlas.sweep(
            kind,
            bits,
            |b, rng| {
                let b = b as u32;
                let first = builder::cuboid(Point3::origin(), ipoint(2, 2, 2)).expect("box");
                let (z_min, z_max) = if coplanar {
                    (int(0), int(2))
                } else {
                    (int(1) + fraction(rng, b), int(3))
                };
                let min = point(int(1) + fraction(rng, b), int(1) + fraction(rng, b), z_min);
                let second = builder::cuboid(min, point(int(3), int(3), z_max)).expect("box");
                (first, second)
            },
            |(a, b)| shaped_bodies(boolean::boolean(a, b, operation)),
        );
    }

    atlas.operation(
        "boolean",
        "near-coincident faces",
        "gap exponent k (faces 2^-k apart)",
        "Boxes [0,2]^3 and [1,3]^2 × [2^-k, 2 + 2^-k]: the horizontal faces approach coincidence as k grows.",
    );
    for (name, operation) in BOOLEAN_OPS {
        atlas.sweep(
            name,
            gaps,
            |k, _| {
                let e = gap(k as u32);
                let first = builder::cuboid(Point3::origin(), ipoint(2, 2, 2)).expect("box");
                let second = builder::cuboid(
                    point(int(1), int(1), e.clone()),
                    point(int(3), int(3), int(2) + e),
                )
                .expect("box");
                (first, second)
            },
            |(a, b)| shaped_bodies(boolean::boolean(a, b, operation)),
        );
    }

    atlas.operation(
        "boolean",
        "planar cut of curved solid",
        "cut-plane bits",
        "Difference of a unit analytic solid and a box above z = h, with h a random rational of the given size crossing the solid (a facing cut).",
    );
    for kind in PRIMITIVES {
        atlas.sweep(
            kind,
            bits,
            |b, rng| {
                let h = cut_base(kind) + fraction(rng, b as u32);
                let cutter =
                    builder::cuboid(point(int(-5), int(-5), h), ipoint(5, 5, 5)).expect("cutter");
                (unit_primitive(kind), cutter)
            },
            |(solid, cutter)| shaped_bodies(boolean::difference(solid, cutter)),
        );
    }

    atlas.operation(
        "boolean",
        "chained union",
        "operand count",
        "Union of a staircase of k + 1 overlapping boxes, as a left fold (CSG depth k) or a balanced tree (depth ceil(log2(k + 1))); the depth atlas reports the tree depth.",
    );
    for kind in ["left fold", "balanced tree"] {
        atlas.sweep(
            kind,
            &operands,
            |k, _| staircase(k as usize),
            |(base, rest)| {
                Shaped::new(
                    tree(base, rest, BooleanOperation::Union, kind == "balanced tree"),
                    chain_shape,
                )
            },
        );
    }

    atlas.operation(
        "boolean",
        "chained difference",
        "operand count",
        "A plate minus k cutters in turn (left fold, CSG depth k): rectangular pockets (planar, concave results) or through-drilled cylinders.",
    );
    for kind in ["rectangular pockets", "cylindrical drills"] {
        atlas.sweep(
            kind,
            &operands,
            |k, _| plate_with_cutters(k as usize, kind == "cylindrical drills"),
            |(plate, cutters)| {
                Shaped::new(
                    tree(plate, cutters, BooleanOperation::Difference, false),
                    chain_shape,
                )
            },
        );
    }

    atlas.operation(
        "boolean",
        "intersection graph",
        "operand sides (n for both)",
        "Exact broad phase, carrier and trim evidence for two crossing n-gon prisms, without face selection or stitching; nodes are exactly intersecting face pairs.",
    );
    for kind in ["convex", "star (concave)"] {
        atlas.sweep(
            kind,
            &sides,
            |n, _| crossing_prisms(n as usize, n as usize, is_star(kind)),
            |(a, b)| {
                Shaped::new(
                    boolean::intersection_graph(solid_of(a), solid_of(b)),
                    graph_shape,
                )
            },
        );
    }
}

fn edits(atlas: &mut Atlas, vertices: &[f64]) {
    atlas.operation(
        "edit",
        "edge split",
        "profile vertices",
        "Split one edge of an n-gon prism at its parameter midpoint, publishing a new validated revision.",
    );
    for kind in ["convex", "star (concave)"] {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| {
                fresh(memo(&mut master, n, || {
                    left_prism(n as usize, is_star(kind))
                }))
            },
            |bodies| {
                let edge = bodies.model().edges().next().expect("an edge");
                let (a, b) = edge.domain().try_real_endpoints().expect("real domain");
                let middle = ((a.clone() + b.clone()) / int(2)).expect("midpoint");
                Shaped::new(
                    edge.split(Parameter::from_real(middle))
                        .map(|split| split.into_model()),
                    revision_shape,
                )
            },
        );
    }

    atlas.operation(
        "edit",
        "face split",
        "profile vertices",
        "Split the cap of a convex n-gon prism along the diagonal between its first and middle vertices.",
    );
    let mut master = None;
    atlas.sweep(
        "convex cap",
        vertices,
        |n, _| fresh(memo(&mut master, n, || left_prism(n as usize, false))),
        |bodies| {
            let face = cap(bodies.model());
            let wire = face.outer().expect("outer wire");
            let starts: Vec<_> = wire
                .edge_uses()
                .map(|edge_use| match edge_use.direction() {
                    Direction::Forward => edge_use.edge().start(),
                    Direction::Reversed => edge_use.edge().end(),
                })
                .collect();
            Shaped::new(
                face.split(starts[0], starts[starts.len() / 2])
                    .map(|split| split.into_model()),
                revision_shape,
            )
        },
    );
}

type Build = fn(usize) -> BodySet;

const QUERY_SOLIDS: [(&str, Build); 3] = [
    ("convex prism", |n| left_prism(n, false)),
    ("star prism", |n| left_prism(n, true)),
    ("revolved polygon", |n| {
        builder::revolve(&polygon(n, false, &int(1), (&int(3), &int(0)), 0.0)).expect("revolved")
    }),
];

fn queries(atlas: &mut Atlas, vertices: &[f64], gaps: &[f64]) {
    atlas.operation(
        "query",
        "volume",
        "profile vertices",
        "Exact solid volume (a rational for prisms, a multiple of π for revolutions) of a fresh revision.",
    );
    for (kind, build) in QUERY_SOLIDS {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || build(n as usize))),
            |bodies| Shaped::new(solid_of(bodies).volume(), real_shape),
        );
    }

    atlas.operation(
        "query",
        "face area",
        "profile vertices",
        "Exact area of the face with the longest outer wire: a prism cap, or a revolved band.",
    );
    for (kind, build) in QUERY_SOLIDS {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || build(n as usize))),
            |bodies| Shaped::new(cap(bodies.model()).area(), real_shape),
        );
    }

    atlas.operation(
        "query",
        "classify point",
        "profile vertices",
        "Exact inside/outside location of a point against a star n-gon prism on a fresh revision; the warm kind reuses one revision and its cached face contours.",
    );
    let inside = || point(q(1, 3), q(1, 7), int(1));
    for (kind, target) in [
        ("inside", inside()),
        ("outside", point(q(1, 3), q(1, 7), int(3))),
    ] {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || left_prism(n as usize, true))),
            |bodies| solid_of(bodies).classify_point(&target),
        );
    }
    atlas.reuse_inputs(true);
    let mut master = None;
    atlas.sweep(
        "inside, warm",
        vertices,
        |n, _| memo(&mut master, n, || left_prism(n as usize, true)).clone(),
        |bodies| solid_of(bodies).classify_point(&inside()),
    );
    atlas.reuse_inputs(false);
    let mut master = None;
    atlas.fuzz(
        "random point",
        &vertices[..5],
        |n, rng| {
            let mut coordinate = |lo: i64, hi: i64| q(rng.range(lo * 1024, hi * 1024), 1024);
            let target = point(coordinate(-5, 5), coordinate(-5, 5), coordinate(-1, 3));
            (
                fresh(memo(&mut master, n, || left_prism(n as usize, true))),
                target,
            )
        },
        |(bodies, target)| solid_of(bodies).classify_point(target),
    );

    atlas.operation(
        "query",
        "classify point near boundary",
        "gap exponent k (point 2^-k inside)",
        "Exact classification of a point 2^-k inside a planar face, a cylinder side, a sphere or a torus.",
    );
    for kind in ["planar face", "cylinder", "sphere", "torus"] {
        atlas.sweep(
            kind,
            gaps,
            |k, _| near_boundary(kind, gap(k as u32)),
            |(bodies, target)| solid_of(bodies).classify_point(target),
        );
    }

    atlas.operation(
        "query",
        "bounds",
        "profile vertices",
        "Exact axis-aligned bounds of a fresh revision.",
    );
    for (kind, build) in QUERY_SOLIDS {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || build(n as usize))),
            |bodies| bodies.model().bounds().map(|bounds| bounds.is_some()),
        );
    }

    atlas.operation(
        "query",
        "rigid transform",
        "profile vertices",
        "Transform a convex n-gon prism into an independent validated revision: integer translation or exact 3-4-5 rotation.",
    );
    for kind in ["translation", "3-4-5 rotation"] {
        let mut master = None;
        let map = if kind == "translation" {
            translation(int(1), int(2), int(3))
        } else {
            rotation()
        };
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || left_prism(n as usize, false))),
            |bodies| shaped_bodies(bodies.transformed(&map)),
        );
    }

    atlas.operation(
        "archive",
        "encode",
        "profile vertices",
        "Deterministic exact JSON encoding of a model.",
    );
    for (kind, build) in QUERY_SOLIDS {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| fresh(memo(&mut master, n, || build(n as usize))),
            |bodies| bodies.model().to_json().map(|json| json.len()),
        );
    }

    atlas.operation(
        "archive",
        "decode and validate",
        "profile vertices",
        "Parse an exact JSON archive under default limits, then replay complete validation into a new model (untrusted-input admission).",
    );
    for (kind, build) in QUERY_SOLIDS {
        let mut master = None;
        atlas.sweep(
            kind,
            vertices,
            |n, _| {
                memo(&mut master, n, || {
                    build(n as usize).model().to_json().expect("archive")
                })
                .clone()
            },
            |json| {
                Shaped::new(
                    RawModel::from_json(json, &ArchiveLimits::default())
                        .and_then(RawModel::validate),
                    revision_shape,
                )
            },
        );
    }
}

fn predicates(atlas: &mut Atlas, gaps: &[f64]) {
    atlas.operation(
        "predicate",
        "surface section near tangency",
        "gap exponent k (2^-k from tangency)",
        "Exact surface/surface intersection whose section degenerates as k grows: a plane 2^-k from tangency with a sphere, cylinder or torus, or two planes 2^-k from parallel.",
    );
    for kind in [
        "plane–sphere",
        "plane–cylinder",
        "plane–torus",
        "plane–plane",
    ] {
        atlas.sweep(
            kind,
            gaps,
            |k, _| near_tangent_surfaces(kind, k as u32),
            |(a, b)| {
                a.intersect_surface(b)
                    .map(|section| format!("{section:?}").len())
            },
        );
    }

    atlas.operation(
        "predicate",
        "line–surface near tangency",
        "gap exponent k (2^-k from tangency)",
        "Exact line/surface contacts for a line 2^-k inside tangency with a unit sphere, cylinder or torus: two nearly coincident roots.",
    );
    for kind in ["line–sphere", "line–cylinder", "line–torus"] {
        atlas.sweep(
            kind,
            gaps,
            |k, _| near_tangent_line(kind, k as u32),
            |(curve, surface)| {
                surface
                    .intersect_curve(curve)
                    .map(|contacts| format!("{contacts:?}").len())
            },
        );
    }

    atlas.operation(
        "predicate",
        "line–line near parallel",
        "gap exponent k (directions 2^-k apart)",
        "Exact segment/segment intersection with nearly parallel directions: crossing, skew (offset in z), or parallel and offset by 2^-k.",
    );
    for kind in ["crossing", "skew", "parallel offset"] {
        atlas.sweep(
            kind,
            gaps,
            |k, _| near_parallel_lines(kind, k as u32),
            |(a, b)| {
                a.intersect_curve(b)
                    .map(|contacts| format!("{contacts:?}").len())
            },
        );
    }
}

#[cfg(feature = "tessellation")]
mod tessellate {
    use std::num::NonZeroUsize;

    use hyperbrep::SurfaceKind;
    use hyperbrep::tessellation::{
        self, ChordalApproximationPolicy, ChordalFaceApproximation, ExactPlanarFaceTriangulation,
    };

    use super::*;

    fn mesh_shape(triangles: usize, points: &[Point3]) -> Shape {
        let (depth, bits) = coordinates(points.iter());
        Shape {
            nodes: triangles as u64,
            depth,
            bits,
        }
    }

    fn exact_mesh_shape<E>(result: &Result<ExactPlanarFaceTriangulation, E>) -> Option<Shape> {
        result
            .as_ref()
            .ok()
            .map(|mesh| mesh_shape(mesh.triangles().len(), mesh.points()))
    }

    fn chordal_shape<E>(result: &Result<ChordalFaceApproximation, E>) -> Option<Shape> {
        result
            .as_ref()
            .ok()
            .map(|mesh| mesh_shape(mesh.triangles().len(), mesh.points()))
    }

    fn first_curved(model: &Model) -> FaceRef<'_> {
        model
            .faces()
            .find(|face| face.surface().geometry().kind() != SurfaceKind::Plane)
            .expect("a curved face")
    }

    fn policy(segments: usize, levels: u8) -> ChordalApproximationPolicy {
        ChordalApproximationPolicy::uniform(
            NonZeroUsize::new(segments).expect("positive segment count"),
            levels,
        )
    }

    pub(super) fn run(atlas: &mut Atlas, vertices: &[f64]) {
        atlas.operation(
            "tessellation",
            "triangulate planar face",
            "profile vertices",
            "Exact triangulation of an n-gon planar sheet face (exact vertices, earcut in the face chart) on a fresh revision.",
        );
        for kind in ["convex", "star (concave)"] {
            let mut master = None;
            atlas.sweep(
                kind,
                vertices,
                |n, _| {
                    memo(&mut master, n, || {
                        sketch(n as usize, is_star(kind)).expect("sketch")
                    })
                    .transformed(&Matrix4::identity())
                    .expect("copy")
                },
                |sheet| {
                    Shaped::new(
                        tessellation::triangulate_planar_face(
                            sheet.faces().next().expect("one face"),
                        ),
                        exact_mesh_shape,
                    )
                },
            );
        }

        atlas.operation(
            "tessellation",
            "chordal approximation by segments",
            "boundary segments per edge use",
            "Chordal mesh of the first curved face (whole sphere, cylinder side, cone side, torus patch) with exact surface-image vertices and no interior refinement.",
        );
        for kind in PRIMITIVES {
            atlas.sweep(
                kind,
                &powers_of_two(0, 7),
                |s, _| (unit_primitive(kind), policy(s as usize, 0)),
                |(bodies, policy)| {
                    Shaped::new(
                        tessellation::approximate_face_chordally(
                            first_curved(bodies.model()),
                            *policy,
                        ),
                        chordal_shape,
                    )
                },
            );
        }

        atlas.operation(
            "tessellation",
            "chordal refinement levels",
            "interior refinement levels",
            "Chordal mesh of the first curved face with 4 boundary segments and k four-way exact-midpoint refinements (4^k triangles per base triangle).",
        );
        for kind in PRIMITIVES {
            atlas.sweep(
                kind,
                &(0..=7).map(f64::from).collect::<Vec<_>>(),
                |k, _| (unit_primitive(kind), policy(4, k as u8)),
                |(bodies, policy)| {
                    Shaped::new(
                        tessellation::approximate_face_chordally(
                            first_curved(bodies.model()),
                            *policy,
                        ),
                        chordal_shape,
                    )
                },
            );
        }
    }
}
