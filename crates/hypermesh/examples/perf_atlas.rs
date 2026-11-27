//! Performance and resource atlas for Hypermesh: exact mesh Booleans (pairs
//! of independently sized operands, every operation, chained and variadic
//! programs, near-coincident faces, coordinate bit lengths and predicate
//! policies), mesh operations (subdivision, Laplacian and Taubin smoothing,
//! exact rotation, convex hull, Minkowski sum and hull, Z slicing, Surface
//! Nets), broad phases (polygon and point BVHs), point and ray queries,
//! validation, and output materialization.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypermesh --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
//!
//! Hypermesh has one scalar type, `hyperreal::Real`, so the scalar axis is
//! the coordinate representation stored in it:
//! - integer: whole coordinates;
//! - dyadic: binary64 coordinates converted exactly (53-bit dyadic rationals);
//! - decimal: coordinates rounded to `n / 10^6`, rationals with non-dyadic
//!   denominators;
//! - irrational: meshes rotated by 30 degrees, whose coordinates are exact
//!   expressions in `sqrt(3)`.
//!
//! The predicate policy (`STRICT`, used by csgrs, or `APPROXIMATE_512`) is a
//! second backend axis. Input kinds: UV spheres (curved, general position),
//! subdivided boxes (many coplanar faces, as in prismatic CAD parts), rotated
//! boxes, nested and disjoint operands, and cubes offset by `2^-k`.
//!
//! Magnitudes are triangle counts unless an axis says otherwise. With
//! `--resources`, mesh results report `Shape { nodes: triangles, depth:
//! deepest coordinate expression, bits: largest stored coordinate rational }`.
//! Chained Boolean programs report the CSG expression depth instead, and BVHs
//! report their node count. Hypermesh has no BSP tree; its broad phase is a
//! BVH whose depth is not public, so BVH shapes report nodes only.
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};
use std::num::NonZeroU32;

use hypermesh::{
    ApproxBounds, BooleanExpression, BooleanMeshBatch, BooleanOp, BooleanProgram, ConvexPolygon,
    ExactBvh, ExactPointBvh, HypermeshResult, MeshContext, MeshOutcome, Plane, Point3,
    PredicatePolicy, Real, SurfaceNetsGrid, Triangle, TriangleMesh, Vector3, boolean,
    boolean_mesh_is_closed, certify_convex_mesh, convex_hull, minkowski_hull, minkowski_sum,
    polygon_soup, surface_nets_standalone,
};
use hyperreal::Rational;
use perf_atlas::{Atlas, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
const APPROX: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

/// UV-sphere longitudes; a sphere with `s` longitudes has `s * (s - 2)` triangles.
const SEGMENTS: [usize; 10] = [6, 8, 12, 16, 24, 32, 48, 64, 96, 128];

fn r(value: i64) -> Real {
    Real::from(value)
}

fn f(value: f64) -> Real {
    Real::try_from(value).expect("finite coordinate")
}

fn ratio(numerator: i64, denominator: i64) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero denominator")
}

/// `2^-k` as an exact rational.
fn pow2_inverse(k: u32) -> Real {
    let mut out = Rational::new(1);
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = &out * &Rational::new(1 << step);
        left -= step;
    }
    (Real::one() / Real::from(out)).expect("nonzero power of two")
}

fn point(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(f(x), f(y), f(z))
}

/// A coordinate converter for one representation kind.
type Coordinate = fn(f64) -> Real;

fn dyadic(value: f64) -> Real {
    f(value)
}

fn decimal(value: f64) -> Real {
    ratio((value * 1e6).round() as i64, 1_000_000)
}

fn integer(value: f64) -> Real {
    r((value * 1024.0).round() as i64)
}

fn sphere_triangles(segments: usize) -> usize {
    segments * (segments - 2)
}

fn segments_for(triangles: f64) -> usize {
    SEGMENTS
        .into_iter()
        .find(|&s| sphere_triangles(s) as f64 == triangles)
        .expect("magnitude is a sphere triangle count")
}

fn sphere_sizes() -> Vec<f64> {
    SEGMENTS
        .iter()
        .map(|&s| sphere_triangles(s) as f64)
        .collect()
}

/// A closed, outward-wound UV sphere with `segments` longitudes and
/// `segments / 2` stacks.
fn sphere_with(
    segments: usize,
    centre: [f64; 3],
    radius: f64,
    coordinate: Coordinate,
) -> TriangleMesh {
    let stacks = (segments / 2).max(2);
    let at = |x: f64, y: f64, z: f64| {
        Point3::new(
            coordinate(centre[0] + radius * x),
            coordinate(centre[1] + radius * y),
            coordinate(centre[2] + radius * z),
        )
    };
    let mut positions = vec![at(0.0, 0.0, 1.0)];
    for i in 1..stacks {
        let phi = PI * i as f64 / stacks as f64;
        for j in 0..segments {
            let theta = TAU * j as f64 / segments as f64;
            positions.push(at(
                phi.sin() * theta.cos(),
                phi.sin() * theta.sin(),
                phi.cos(),
            ));
        }
    }
    positions.push(at(0.0, 0.0, -1.0));
    let bottom = positions.len() - 1;
    let v = |i: usize, j: usize| 1 + (i - 1) * segments + j % segments;
    let mut triangles = Vec::with_capacity(sphere_triangles(segments));
    for j in 0..segments {
        triangles.push(Triangle::new(0, v(1, j), v(1, j + 1)));
    }
    for i in 1..stacks - 1 {
        for j in 0..segments {
            let (a, b, c, d) = (v(i, j), v(i, j + 1), v(i + 1, j), v(i + 1, j + 1));
            triangles.push(Triangle::new(a, c, d));
            triangles.push(Triangle::new(a, d, b));
        }
    }
    for j in 0..segments {
        triangles.push(Triangle::new(
            bottom,
            v(stacks - 1, j + 1),
            v(stacks - 1, j),
        ));
    }
    TriangleMesh::new(positions, triangles)
}

fn sphere(segments: usize, centre: [f64; 3], radius: f64) -> TriangleMesh {
    sphere_with(segments, centre, radius, dyadic)
}

fn cuboid(min: [Real; 3], max: [Real; 3]) -> TriangleMesh {
    let corner = |x: bool, y: bool, z: bool| {
        let pick = |high: bool, axis: usize| {
            if high {
                max[axis].clone()
            } else {
                min[axis].clone()
            }
        };
        Point3::new(pick(x, 0), pick(y, 1), pick(z, 2))
    };
    TriangleMesh::new(
        vec![
            corner(false, false, false),
            corner(true, false, false),
            corner(true, true, false),
            corner(false, true, false),
            corner(false, false, true),
            corner(true, false, true),
            corner(true, true, true),
            corner(false, true, true),
        ],
        [
            [4, 5, 6],
            [4, 6, 7],
            [0, 3, 2],
            [0, 2, 1],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
            [3, 7, 6],
            [3, 6, 2],
            [0, 1, 5],
            [0, 5, 4],
        ]
        .into_iter()
        .map(|[a, b, c]| Triangle::new(a, b, c))
        .collect(),
    )
}

fn cube(centre: [Real; 3], half: &Real) -> TriangleMesh {
    let min = centre.clone().map(|c| &c - half);
    let max = centre.map(|c| &c + half);
    cuboid(min, max)
}

/// A cube whose faces are subdivided `levels` times: `12 * 4^levels` triangles.
fn boxed(levels: u32, centre: [Real; 3], half: &Real) -> TriangleMesh {
    let mesh = cube(centre, half);
    match NonZeroU32::new(levels) {
        Some(levels) => mesh.subdivide_triangles(levels).expect("valid cube"),
        None => mesh,
    }
}

fn box_sizes(max_level: u32) -> Vec<f64> {
    (0..=max_level)
        .map(|l| (12 * 4usize.pow(l)) as f64)
        .collect()
}

fn box_level(triangles: f64) -> u32 {
    ((triangles / 12.0).log2() / 2.0).round() as u32
}

fn rotated(mesh: &TriangleMesh, degrees: i64) -> TriangleMesh {
    mesh.try_rotated_xyz_degrees(&STRICT, r(0), r(0), r(degrees))
        .expect("rotation of a finite mesh")
        .into_value()
}

/// A mesh with the same coordinates and indices but no retained facts, so a
/// timed call never reuses validation cached by an earlier call.
fn fresh(mesh: &TriangleMesh) -> TriangleMesh {
    TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec())
}

/// Build an input once per magnitude, then hand out fresh copies of it.
fn memo<V: Clone>(cache: &mut HashMap<u64, V>, key: f64, build: impl FnOnce() -> V) -> V {
    cache.entry(key.to_bits()).or_insert_with(build).clone()
}

fn coordinate_shape<'a>(points: impl IntoIterator<Item = &'a Point3>) -> (u64, u64) {
    points
        .into_iter()
        .flat_map(|p| [&p.x, &p.y, &p.z])
        .map(|c| c.representation_size(1 << 12))
        .fold((0, 0), |(depth, bits), size| {
            (
                depth.max(size.depth as u64),
                bits.max(size.max_rational_bits),
            )
        })
}

fn mesh_shape(mesh: &TriangleMesh) -> Shape {
    let (depth, bits) = coordinate_shape(mesh.positions.iter());
    Shape {
        nodes: mesh.triangles.len() as u64,
        depth,
        bits,
    }
}

type MeshResult = HypermeshResult<TriangleMesh>;

fn shaped_mesh(result: MeshResult) -> Shaped<MeshResult> {
    Shaped::new(result, |result| result.as_ref().ok().map(mesh_shape))
}

fn shaped_outcome(result: HypermeshResult<MeshOutcome<TriangleMesh>>) -> Shaped<MeshResult> {
    shaped_mesh(result.map(MeshOutcome::into_value))
}

fn batch_shape(batch: &BooleanMeshBatch) -> Shape {
    let (depth, bits) = coordinate_shape(batch.vertices.iter());
    Shape {
        nodes: batch.results.iter().map(|r| r.triangles.len() as u64).sum(),
        depth,
        bits,
    }
}

type BatchResult = HypermeshResult<BooleanMeshBatch>;

fn shaped_batch(result: BatchResult) -> Shaped<BatchResult> {
    Shaped::new(result, |result| result.as_ref().ok().map(batch_shape))
}

/// A Boolean result paired with the depth of the CSG program that made it.
type Chained<T> = (HypermeshResult<T>, u64);

fn chained_batch(result: Chained<BooleanMeshBatch>) -> Shaped<Chained<BooleanMeshBatch>> {
    Shaped::new(result, |(result, depth)| {
        result.as_ref().ok().map(|batch| Shape {
            depth: *depth,
            ..batch_shape(batch)
        })
    })
}

fn chained_mesh(result: Chained<TriangleMesh>) -> Shaped<Chained<TriangleMesh>> {
    Shaped::new(result, |(result, depth)| {
        result.as_ref().ok().map(|mesh| Shape {
            depth: *depth,
            ..mesh_shape(mesh)
        })
    })
}

fn pair(context: &MeshContext, a: &TriangleMesh, b: &TriangleMesh, op: BooleanOp) -> BatchResult {
    boolean(
        context,
        &[a.as_ref(), b.as_ref()],
        BooleanProgram::Operation(op),
    )
    .map(MeshOutcome::into_value)
}

fn first_mesh(batch: BatchResult) -> MeshResult {
    batch?
        .into_triangle_meshes()?
        .pop()
        .ok_or(hypermesh::HypermeshError::EmptyInput)
}

/// A topologically ordered truth program and its root.
#[derive(Clone)]
struct Program {
    nodes: Vec<BooleanExpression>,
    root: u32,
}

impl Program {
    fn push(&mut self, node: BooleanExpression) -> u32 {
        self.nodes.push(node);
        self.root = self.nodes.len() as u32 - 1;
        self.root
    }

    /// Nodes on the longest path from the root to an operand.
    fn depth(&self) -> u64 {
        let mut depth: Vec<u64> = Vec::with_capacity(self.nodes.len());
        for node in &self.nodes {
            let d: u64 = match *node {
                BooleanExpression::Not(a) => depth[a as usize] + 1,
                BooleanExpression::And([a, b])
                | BooleanExpression::Or([a, b])
                | BooleanExpression::Xor([a, b]) => depth[a as usize].max(depth[b as usize]) + 1,
                _ => 1,
            };
            depth.push(d);
        }
        depth[self.root as usize]
    }

    /// `((A - B1) - B2) - ...`, one operation per tool, as eager CAD history.
    fn linear_difference(tools: usize) -> Self {
        let mut program = Self {
            nodes: Vec::new(),
            root: 0,
        };
        let mut acc = program.push(BooleanExpression::Operand(0));
        for tool in 1..=tools as u32 {
            let operand = program.push(BooleanExpression::Operand(tool));
            let outside = program.push(BooleanExpression::Not(operand));
            acc = program.push(BooleanExpression::And([acc, outside]));
        }
        program.root = acc;
        program
    }

    /// `A - (B1 | B2 | ...)` with a balanced union tree.
    fn balanced_difference(tools: usize) -> Self {
        let mut program = Self {
            nodes: Vec::new(),
            root: 0,
        };
        let body = program.push(BooleanExpression::Operand(0));
        let mut level: Vec<u32> = (1..=tools as u32)
            .map(|tool| program.push(BooleanExpression::Operand(tool)))
            .collect();
        while level.len() > 1 {
            level = level
                .chunks(2)
                .map(|pair| match *pair {
                    [a, b] => program.push(BooleanExpression::Or([a, b])),
                    [a] => a,
                    _ => unreachable!("chunks of two"),
                })
                .collect();
        }
        let outside = program.push(BooleanExpression::Not(level[0]));
        program.push(BooleanExpression::And([body, outside]));
        program
    }

    /// The built-in variadic difference: the first operand minus all others.
    fn variadic_difference() -> Self {
        Self {
            nodes: vec![BooleanExpression::Operation(BooleanOp::Difference)],
            root: 0,
        }
    }
}

/// A 32-unit block and `tools` cutting tools on a grid over its top face:
/// through-holes (boxes) or spherical pockets.
fn drilling(tools: usize, pockets: bool) -> Vec<TriangleMesh> {
    let mut meshes = vec![cube([r(0), r(0), r(0)], &r(16))];
    let side = (tools as f64).sqrt().ceil().max(1.0) as usize;
    let spacing = 24.0 / side as f64;
    for i in 0..tools {
        let x = -12.0 + spacing * ((i % side) as f64 + 0.5);
        let y = -12.0 + spacing * ((i / side) as f64 + 0.5);
        meshes.push(if pockets {
            sphere(12, [x, y, 16.0], spacing * 0.35)
        } else {
            let half = spacing * 0.3;
            cuboid(
                [f(x - half), f(y - half), r(-20)],
                [f(x + half), f(y + half), r(20)],
            )
        });
    }
    meshes
}

/// Sphere A at the origin and a smaller sphere B in general position.
fn sphere_pair(a: usize, b: usize, coordinate: Coordinate) -> [TriangleMesh; 2] {
    [
        sphere_with(a, [0.0, 0.0, 0.0], 1.0, coordinate),
        sphere_with(b, [0.43, 0.31, 0.27], 0.9, coordinate),
    ]
}

/// Two operands of about `triangles` triangles each, of one input kind.
fn operands(kind: &str, triangles: f64) -> [TriangleMesh; 2] {
    let level = box_level(triangles);
    let half = r(1);
    match kind {
        "spheres (dyadic)" => {
            let s = segments_for(triangles);
            sphere_pair(s, s, dyadic)
        }
        "spheres (decimal)" => {
            let s = segments_for(triangles);
            sphere_pair(s, s, decimal)
        }
        "spheres (integer)" => {
            let s = segments_for(triangles);
            sphere_pair(s, s, integer)
        }
        "boxes (coplanar grid)" => [
            boxed(level, [r(0), r(0), r(0)], &half),
            boxed(level, [ratio(1, 2), ratio(1, 3), ratio(1, 5)], &half),
        ],
        "rotated boxes (irrational)" => [
            boxed(level, [r(0), r(0), r(0)], &half),
            rotated(
                &boxed(level, [ratio(1, 2), ratio(1, 3), ratio(1, 5)], &half),
                30,
            ),
        ],
        "nested spheres" => {
            let s = segments_for(triangles);
            [
                sphere(s, [0.0, 0.0, 0.0], 1.0),
                sphere(s, [0.05, 0.02, 0.01], 0.5),
            ]
        }
        "disjoint spheres" => {
            let s = segments_for(triangles);
            [
                sphere(s, [0.0, 0.0, 0.0], 1.0),
                sphere(s, [3.0, 0.0, 0.0], 1.0),
            ]
        }
        other => unreachable!("unknown operand kind {other}"),
    }
}

fn is_box_kind(kind: &str) -> bool {
    kind.contains("boxes")
}

fn booleans(atlas: &mut Atlas) {
    let spheres = sphere_sizes();
    let boxes = box_sizes(5);
    let grid_spheres: Vec<f64> = spheres[..8].to_vec();

    atlas.operation(
        "boolean",
        "union grid",
        "triangles in A",
        "Exact union of two curved UV spheres in general position whose triangle counts vary independently; one shared arrangement, closure-certified output.",
    );
    atlas.grid(
        "sphere ∪ sphere",
        ("triangles in A", &grid_spheres),
        ("triangles in B", &grid_spheres),
        |a, b, _| sphere_pair(segments_for(a), segments_for(b), dyadic),
        |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
    );
    atlas.operation(
        "boolean",
        "difference grid",
        "triangles in the box",
        "A subdivided box (prismatic, coplanar-heavy) minus a UV sphere, sizes varied independently: a curved pocket in a faceted part.",
    );
    let mut cache = HashMap::new();
    atlas.grid(
        "box − sphere",
        ("triangles in the box", &boxes),
        ("triangles in the sphere", &grid_spheres),
        |a, b, _| {
            let block = memo(&mut cache, a, || {
                boxed(box_level(a), [r(0), r(0), r(0)], &r(1))
            });
            (
                fresh(&block),
                sphere(segments_for(b), [0.6, 0.55, 0.5], 0.8),
            )
        },
        |(a, b)| shaped_batch(pair(&STRICT, a, b, BooleanOp::Difference)),
    );

    atlas.operation(
        "boolean",
        "by operation",
        "triangles per operand",
        "Each Boolean operation on two overlapping UV spheres of equal size.",
    );
    for (kind, op) in [
        ("union", BooleanOp::Union),
        ("intersection", BooleanOp::Intersection),
        ("difference", BooleanOp::Difference),
        ("symmetric difference", BooleanOp::SymmetricDifference),
    ] {
        atlas.sweep(
            kind,
            &spheres,
            |m, _| operands("spheres (dyadic)", m),
            move |[a, b]| shaped_batch(pair(&STRICT, a, b, op)),
        );
    }

    atlas.operation(
        "boolean",
        "union by coordinate kind",
        "triangles per operand",
        "Union of two overlapping spheres whose coordinates are integers, binary64 dyadics or decimal rationals: the scalar representation axis.",
    );
    for kind in ["spheres (integer)", "spheres (dyadic)", "spheres (decimal)"] {
        atlas.sweep(
            kind,
            &spheres,
            |m, _| operands(kind, m),
            |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
        );
    }

    atlas.operation(
        "boolean",
        "union by configuration",
        "triangles per operand",
        "Union across operand configurations: offset subdivided boxes (coplanar grids), a box rotated by 30 degrees (irrational coordinates), nested and disjoint spheres.",
    );
    for kind in [
        "boxes (coplanar grid)",
        "rotated boxes (irrational)",
        "nested spheres",
        "disjoint spheres",
    ] {
        let sizes = if is_box_kind(kind) { &boxes } else { &spheres };
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            sizes,
            |m, _| {
                let [a, b] = memo(&mut cache, m, || operands(kind, m));
                [fresh(&a), fresh(&b)]
            },
            |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
        );
    }

    atlas.operation(
        "boolean",
        "union by predicate policy",
        "triangles per operand",
        "Union under STRICT (certified decisions only) and APPROXIMATE_512 (may consume the 512-bit terminal decision).",
    );
    for (policy, context) in [("STRICT", STRICT), ("APPROXIMATE_512", APPROX)] {
        for kind in ["spheres (dyadic)", "rotated boxes (irrational)"] {
            let sizes = if is_box_kind(kind) { &boxes } else { &spheres };
            let mut cache = HashMap::new();
            atlas.sweep(
                &format!("{policy}, {kind}"),
                sizes,
                |m, _| {
                    let [a, b] = memo(&mut cache, m, || operands(kind, m));
                    [fresh(&a), fresh(&b)]
                },
                move |[a, b]| shaped_batch(pair(&context, a, b, BooleanOp::Union)),
            );
        }
    }

    let exponents: Vec<f64> = (0..=64).step_by(4).map(f64::from).collect();
    atlas.operation(
        "boolean",
        "near-coincident faces",
        "offset exponent k (offset 2^-k)",
        "Union of two unit cubes offset by 2^-k, on every axis or along x only (two exactly coplanar face pairs): distance to degeneracy.",
    );
    for (kind, axes) in [
        ("offset on x, y, z", [true; 3]),
        ("offset on x only", [true, false, false]),
    ] {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| {
                let d = pow2_inverse(k as u32);
                let centre = axes.map(|on| if on { d.clone() } else { r(0) });
                [cube([r(0), r(0), r(0)], &r(1)), cube(centre, &r(1))]
            },
            |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
        );
    }
    atlas.fuzz(
        "random offset per axis",
        &exponents,
        |k, rng| {
            let centre = [0, 1, 2].map(|_| {
                let e = (k as i64 + rng.range(-2, 2)).max(0) as u32;
                &pow2_inverse(e) * &r(rng.range(-3, 3))
            });
            [cube([r(0), r(0), r(0)], &r(1)), cube(centre, &r(1))]
        },
        |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
    );

    atlas.operation(
        "boolean",
        "coordinate bits",
        "perturbation bits b (jitter 2^-b)",
        "Union of two 120-triangle spheres whose every coordinate is jittered by j * 2^-b, |j| <= 8: cost of long exact coordinates.",
    );
    let bits = powers_of_two(3, 11);
    atlas.sweep(
        "jittered spheres",
        &bits,
        |b, rng| {
            let eps = pow2_inverse(b as u32);
            sphere_pair(12, 12, dyadic).map(|mesh| {
                let positions = mesh
                    .positions
                    .iter()
                    .map(|p| {
                        let mut jitter = |c: &Real| c + &(&eps * &r(rng.range(-8, 8)));
                        Point3::new(jitter(&p.x), jitter(&p.y), jitter(&p.z))
                    })
                    .collect();
                TriangleMesh::new(positions, mesh.triangles.to_vec())
            })
        },
        |[a, b]| shaped_batch(pair(&STRICT, a, b, BooleanOp::Union)),
    );

    let counts = powers_of_two(1, 7);
    atlas.operation(
        "chained boolean",
        "variadic union",
        "operand count",
        "One variadic union of k overlapping 120-triangle spheres in a row, from one shared arrangement.",
    );
    atlas.sweep(
        "spheres in a row",
        &counts,
        |k, _| {
            (0..k as usize)
                .map(|i| sphere(12, [0.7 * i as f64, 0.1 * (i % 3) as f64, 0.0], 0.5))
                .collect::<Vec<_>>()
        },
        |meshes| {
            let views: Vec<_> = meshes.iter().map(TriangleMesh::as_ref).collect();
            shaped_batch(
                boolean(&STRICT, &views, BooleanProgram::Operation(BooleanOp::Union))
                    .map(MeshOutcome::into_value),
            )
        },
    );

    atlas.operation(
        "chained boolean",
        "difference program",
        "cutting tools",
        "A block minus k through-holes evaluated as one Boolean program: a linear chain ((A - B1) - B2)..., a balanced union of tools, or the built-in variadic difference. Shape depth is the program depth.",
    );
    type Build = fn(usize) -> Program;
    let programs: [(&str, Build); 3] = [
        ("linear chain", Program::linear_difference),
        ("balanced tree", Program::balanced_difference),
        ("variadic difference", |_| Program::variadic_difference()),
    ];
    for (kind, build) in programs {
        atlas.sweep(
            kind,
            &counts,
            |k, _| (drilling(k as usize, false), build(k as usize)),
            |(meshes, program)| {
                let views: Vec<_> = meshes.iter().map(TriangleMesh::as_ref).collect();
                let result = boolean(
                    &STRICT,
                    &views,
                    BooleanProgram::Expressions {
                        nodes: &program.nodes,
                        roots: &[program.root],
                    },
                )
                .map(MeshOutcome::into_value);
                chained_batch((result, program.depth()))
            },
        );
    }
    atlas.sweep(
        "linear chain, spherical pockets",
        &counts,
        |k, _| {
            (
                drilling(k as usize, true),
                Program::linear_difference(k as usize),
            )
        },
        |(meshes, program)| {
            let views: Vec<_> = meshes.iter().map(TriangleMesh::as_ref).collect();
            let result = boolean(
                &STRICT,
                &views,
                BooleanProgram::Expressions {
                    nodes: &program.nodes,
                    roots: &[program.root],
                },
            )
            .map(MeshOutcome::into_value);
            chained_batch((result, program.depth()))
        },
    );

    atlas.operation(
        "chained boolean",
        "eager differences",
        "cutting tools",
        "A block minus k tools one Boolean at a time, each result materialized and fed to the next (eager CAD history); compare with the one-program evaluation. Shape depth is the chain length.",
    );
    for (kind, pockets) in [("through-holes", false), ("spherical pockets", true)] {
        atlas.sweep(
            kind,
            &counts,
            |k, _| drilling(k as usize, pockets),
            |meshes| {
                let mut acc = Ok(meshes[0].clone());
                for tool in &meshes[1..] {
                    acc = acc.and_then(|body| {
                        first_mesh(pair(&STRICT, &body, tool, BooleanOp::Difference))
                    });
                }
                chained_mesh((acc, meshes.len() as u64 - 1))
            },
        );
    }

    atlas.operation(
        "chained boolean",
        "eager unions",
        "operands",
        "Accumulate k overlapping spheres one union at a time; the growing result is re-validated and re-arranged at every step.",
    );
    atlas.sweep(
        "spheres in a row",
        &counts,
        |k, _| {
            (0..k as usize)
                .map(|i| sphere(12, [0.7 * i as f64, 0.1 * (i % 3) as f64, 0.0], 0.5))
                .collect::<Vec<_>>()
        },
        |meshes| {
            let mut acc = Ok(meshes[0].clone());
            for next in &meshes[1..] {
                acc = acc.and_then(|body| first_mesh(pair(&STRICT, &body, next, BooleanOp::Union)));
            }
            chained_mesh((acc, meshes.len() as u64 - 1))
        },
    );
}

fn mesh_operations(atlas: &mut Atlas) {
    let spheres = sphere_sizes();
    let levels: Vec<f64> = (1..=7).map(f64::from).collect();
    atlas.operation(
        "mesh",
        "subdivide",
        "subdivision levels",
        "Split every triangle into four, level times, with exact edge midpoints; 12-triangle cubes, a 48-triangle sphere, and a cube rotated by 30 degrees.",
    );
    for kind in [
        "cube (integer)",
        "sphere (dyadic)",
        "rotated cube (irrational)",
    ] {
        let base = match kind {
            "cube (integer)" => cube([r(0), r(0), r(0)], &r(1)),
            "sphere (dyadic)" => sphere(8, [0.0; 3], 1.0),
            _ => rotated(&cube([r(0), r(0), r(0)], &r(1)), 30),
        };
        atlas.sweep(
            kind,
            &levels,
            |l, _| (fresh(&base), NonZeroU32::new(l as u32).expect("level >= 1")),
            |(mesh, levels)| shaped_mesh(mesh.subdivide_triangles(*levels)),
        );
    }

    let iterations = powers_of_two(0, 6);
    atlas.operation(
        "mesh",
        "laplacian smooth by iterations",
        "iterations",
        "Exact Laplacian smoothing of a 48-triangle sphere; coordinates are exact rationals, so stored bits grow with every iteration.",
    );
    for (kind, lambda) in [("lambda 1/2", ratio(1, 2)), ("lambda 1/3", ratio(1, 3))] {
        let base = sphere(8, [0.0; 3], 1.0);
        atlas.sweep(
            kind,
            &iterations,
            |n, _| (fresh(&base), n as usize),
            |(mesh, n)| shaped_mesh(mesh.laplacian_smooth(&lambda, *n)),
        );
    }
    atlas.operation(
        "mesh",
        "taubin smooth by iterations",
        "iterations",
        "Exact Taubin smoothing (lambda 1/2, mu -53/100) of a 48-triangle sphere.",
    );
    let (lambda, mu) = (ratio(1, 2), ratio(-53, 100));
    let base = sphere(8, [0.0; 3], 1.0);
    atlas.sweep(
        "sphere",
        &iterations,
        |n, _| (fresh(&base), n as usize),
        |(mesh, n)| shaped_mesh(mesh.taubin_smooth(&lambda, &mu, *n)),
    );
    atlas.operation(
        "mesh",
        "laplacian smooth by size",
        "triangles",
        "Two exact Laplacian iterations (lambda 1/2) over spheres of growing size.",
    );
    let half = ratio(1, 2);
    atlas.sweep(
        "sphere",
        &spheres,
        |m, _| sphere(segments_for(m), [0.0; 3], 1.0),
        |mesh| shaped_mesh(mesh.laplacian_smooth(&half, 2)),
    );

    atlas.operation(
        "mesh",
        "rotate",
        "triangles",
        "Exact rotation about z by 90 degrees (rational image), 30 degrees (sqrt(3)) and 7 degrees (transcendental), with certified validity checks.",
    );
    for degrees in [90, 30, 7] {
        atlas.sweep(
            &format!("{degrees} degrees"),
            &spheres,
            |m, _| sphere(segments_for(m), [0.0; 3], 1.0),
            move |mesh| {
                shaped_outcome(mesh.try_rotated_xyz_degrees(&STRICT, r(0), r(0), r(degrees)))
            },
        );
    }

    let points = powers_of_two(4, 15);
    atlas.operation(
        "mesh",
        "convex hull",
        "points",
        "Exact 3D convex hull: random integer points in a cube, binary64 points on a sphere (all extreme), exactly cospherical rational points, and an integer lattice (many coplanar points).",
    );
    for kind in [
        "uniform cube (integer)",
        "on sphere (dyadic)",
        "cospherical (rational)",
        "lattice (coplanar)",
    ] {
        atlas.sweep(
            kind,
            &points,
            |n, rng| hull_points(kind, n as usize, rng),
            |points| shaped_outcome(convex_hull(&STRICT, points)),
        );
    }

    let ball_points = powers_of_two(2, 7);
    atlas.operation(
        "mesh",
        "minkowski sum grid",
        "hull points in A",
        "Regularized Minkowski sum of two exactly convex balls (exact hulls of n Fibonacci-sphere points) whose sizes vary independently: a ball-end tool around a convex part. Binary64 UV spheres are not exactly convex and take the general path; two 24-triangle ones took 31 s, so they are left out.",
    );
    atlas.grid(
        "ball ⊕ ball",
        ("hull points in A", &ball_points),
        ("hull points in B", &ball_points),
        |a, b, _| {
            (
                convex_ball(a as usize, [0.0; 3], 1.0),
                convex_ball(b as usize, [0.3, 0.2, 0.1], 0.5),
            )
        },
        |(a, b)| shaped_outcome(minkowski_sum(&STRICT, a, b)),
    );
    atlas.operation(
        "mesh",
        "minkowski sum (nonconvex)",
        "hull points in the ball",
        "Minkowski sum of a nonconvex L-bracket (a union of two boxes) with an exactly convex ball of growing size: the rounding-offset workflow.",
    );
    let bracket = first_mesh(pair(
        &STRICT,
        &cuboid([r(0), r(0), r(0)], [r(4), r(1), r(1)]),
        &cuboid([r(0), r(0), r(0)], [r(1), r(4), r(1)]),
        BooleanOp::Union,
    ))
    .expect("L-bracket union");
    atlas.sweep(
        "L-bracket ⊕ ball",
        &ball_points,
        |m, _| (fresh(&bracket), convex_ball(m as usize, [0.0; 3], 0.25)),
        |(a, b)| shaped_outcome(minkowski_sum(&STRICT, a, b)),
    );
    atlas.operation(
        "mesh",
        "minkowski hull",
        "points per operand",
        "Convex hull of all pairwise point sums of two random point clouds of equal size.",
    );
    let hull_sizes = powers_of_two(2, 9);
    atlas.sweep(
        "on sphere (dyadic)",
        &hull_sizes,
        |n, rng| {
            (
                hull_points("on sphere (dyadic)", n as usize, rng),
                hull_points("on sphere (dyadic)", n as usize, rng),
            )
        },
        |(a, b)| shaped_outcome(minkowski_hull(&STRICT, a, b)),
    );

    atlas.operation(
        "slice",
        "slice z",
        "triangles",
        "One horizontal section: a sphere at a generic height, a subdivided box at a height through its grid vertices (on-plane vertices and edges), and a rotated box (irrational).",
    );
    let boxes = box_sizes(6);
    for kind in ["sphere", "box through vertices", "rotated box (irrational)"] {
        let sizes = if kind == "sphere" { &spheres } else { &boxes };
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            sizes,
            |m, _| {
                let mesh = memo(&mut cache, m, || match kind {
                    "sphere" => sphere(segments_for(m), [0.0; 3], 1.0),
                    "box through vertices" => boxed(box_level(m), [r(0), r(0), r(0)], &r(1)),
                    _ => rotated(&boxed(box_level(m), [r(0), r(0), r(0)], &r(1)), 30),
                });
                let z = if kind == "sphere" { f(0.1234) } else { r(0) };
                (fresh(&mesh), z)
            },
            |(mesh, z)| {
                let section = mesh.slice_z(&STRICT, z).map(MeshOutcome::into_value);
                Shaped::new(section, |section| {
                    section.as_ref().ok().map(|s| Shape {
                        nodes: s.segments.len() as u64,
                        depth: (s.above.len() + s.below.len() + s.wires.len()) as u64,
                        bits: 0,
                    })
                })
            },
        );
    }
    atlas.operation(
        "slice",
        "slice stack",
        "layers",
        "Slice a 2208-triangle sphere into evenly spaced layers, as a CAM or additive-manufacturing slicer does; shape nodes are total segments.",
    );
    let layers = powers_of_two(0, 9);
    let body = sphere(48, [0.0; 3], 1.0);
    atlas.sweep(
        "sphere",
        &layers,
        |n, _| (fresh(&body), n as usize),
        |(mesh, layers)| {
            let mut segments = Vec::with_capacity(*layers);
            for i in 0..*layers {
                let z = -1.0 + 2.0 * (i as f64 + 0.5) / *layers as f64;
                segments.push(
                    mesh.slice_z(&STRICT, &f(z))
                        .map(|s| s.into_value().segments.len() as u64),
                );
            }
            Shaped::new(segments, |segments| {
                Some(Shape {
                    nodes: segments.iter().flatten().sum(),
                    depth: 0,
                    bits: 0,
                })
            })
        },
    );

    atlas.operation(
        "mesh",
        "surface nets",
        "grid samples per axis",
        "Surface Nets over an n^3 grid of an integer sphere field and a binary64 gyroid field.",
    );
    let resolutions = powers_of_two(3, 7);
    for kind in ["sphere (integer field)", "gyroid (dyadic field)"] {
        atlas.sweep(
            kind,
            &resolutions,
            |n, _| field(kind, n as u32),
            |(origin, step, n, values)| {
                let grid = SurfaceNetsGrid::new(origin, step, [*n; 3], values);
                let out = surface_nets_standalone(&STRICT, grid).map(|o| o.into_value().mesh);
                Shaped::new(out, |out| out.as_ref().ok().map(mesh_shape))
            },
        );
    }
}

/// An exactly convex ball: the exact hull of `n` Fibonacci-sphere points.
fn convex_ball(n: usize, centre: [f64; 3], radius: f64) -> TriangleMesh {
    let golden = PI * (3.0 - 5f64.sqrt());
    let points: Vec<Point3> = (0..n)
        .map(|i| {
            let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
            let ring = (1.0 - z * z).sqrt();
            let theta = golden * i as f64;
            point(
                centre[0] + radius * ring * theta.cos(),
                centre[1] + radius * ring * theta.sin(),
                centre[2] + radius * z,
            )
        })
        .collect();
    convex_hull(&STRICT, &points)
        .expect("nondegenerate points")
        .into_value()
}

fn hull_points(kind: &str, n: usize, rng: &mut Rng) -> Vec<Point3> {
    match kind {
        "uniform cube (integer)" => (0..n)
            .map(|_| {
                Point3::new(
                    r(rng.range(0, 1 << 20)),
                    r(rng.range(0, 1 << 20)),
                    r(rng.range(0, 1 << 20)),
                )
            })
            .collect(),
        "on sphere (dyadic)" => (0..n)
            .map(|_| {
                loop {
                    let v = [0; 3].map(|_| rng.unit() * 2.0 - 1.0);
                    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
                    if norm > 0.1 && norm <= 1.0 {
                        break point(v[0] / norm, v[1] / norm, v[2] / norm);
                    }
                }
            })
            .collect(),
        "cospherical (rational)" => {
            // Inverse stereographic projection of rational (a/D, b/D).
            let d: i64 = 1 << 10;
            (0..n)
                .map(|_| {
                    let (a, b) = (rng.range(-4 * d, 4 * d), rng.range(-4 * d, 4 * d));
                    let w = a * a + b * b + d * d;
                    Point3::new(
                        ratio(2 * a * d, w),
                        ratio(2 * b * d, w),
                        ratio(a * a + b * b - d * d, w),
                    )
                })
                .collect()
        }
        "lattice (coplanar)" => {
            let side = (n as f64).cbrt().ceil() as i64;
            (0..n as i64)
                .map(|i| Point3::new(r(i % side), r(i / side % side), r(i / (side * side))))
                .collect()
        }
        other => unreachable!("unknown point kind {other}"),
    }
}

type Field = (Point3, Vector3, u32, Vec<Real>);

fn field(kind: &str, n: u32) -> Field {
    let mut values = Vec::with_capacity((n * n * n) as usize);
    let c = i64::from(n) - 1;
    let radius = (0.8 * c as f64).round() as i64;
    for z in 0..i64::from(n) {
        for y in 0..i64::from(n) {
            for x in 0..i64::from(n) {
                values.push(if kind.starts_with("sphere") {
                    let (x, y, z) = (2 * x - c, 2 * y - c, 2 * z - c);
                    r(x * x + y * y + z * z - radius * radius)
                } else {
                    let s = TAU * 2.0 / f64::from(n);
                    let (x, y, z) = (x as f64 * s, y as f64 * s, z as f64 * s);
                    f(x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos())
                });
            }
        }
    }
    (
        Point3::new(r(0), r(0), r(0)),
        Vector3::from_xyz(r(1), r(1), r(1)),
        n,
        values,
    )
}

fn soup_polygons(mesh: &TriangleMesh) -> Vec<ConvexPolygon> {
    polygon_soup(&STRICT, &[mesh.as_ref()])
        .expect("valid sphere")
        .into_value()
        .polygons
}

fn bvh_shape(bvh: &HypermeshResult<ExactBvh>) -> Option<Shape> {
    bvh.as_ref().ok().map(|bvh| Shape {
        nodes: bvh.node_count() as u64,
        depth: 0,
        bits: 0,
    })
}

fn queries(atlas: &mut Atlas) {
    let spheres = sphere_sizes();
    atlas.operation(
        "broad phase",
        "polygon bvh build",
        "triangles",
        "Build the exact polygon BVH over a sphere's certified polygons; shape nodes are BVH nodes.",
    );
    let mut cache = HashMap::new();
    atlas.sweep(
        "sphere",
        &spheres,
        |m, _| {
            memo(&mut cache, m, || {
                soup_polygons(&sphere(segments_for(m), [0.0; 3], 1.0))
            })
        },
        |polygons| {
            Shaped::new(
                ExactBvh::build(&STRICT, polygons).map(MeshOutcome::into_value),
                bvh_shape,
            )
        },
    );
    atlas.operation(
        "broad phase",
        "bvh pair query",
        "triangles per operand",
        "All overlapping primitive pairs between the BVHs of two overlapping spheres (the Boolean broad phase).",
    );
    let mut cache = HashMap::new();
    atlas.sweep(
        "sphere pair",
        &spheres,
        |m, _| {
            memo(&mut cache, m, || {
                let [a, b] = sphere_pair(segments_for(m), segments_for(m), dyadic);
                let build = |mesh: &TriangleMesh| {
                    ExactBvh::build(&STRICT, &soup_polygons(mesh))
                        .expect("valid polygons")
                        .into_value()
                };
                (build(&a), build(&b))
            })
        },
        |(a, b)| {
            let mut pairs = 0u64;
            a.intersect_pairs(&STRICT, b, |_, _| pairs += 1)
                .map(|_| pairs)
        },
    );
    atlas.operation(
        "broad phase",
        "bvh box query",
        "triangles",
        "Primitives whose bounds overlap a small box on the surface of a sphere.",
    );
    let mut cache = HashMap::new();
    atlas.sweep(
        "sphere",
        &spheres,
        |m, _| {
            let bvh = memo(&mut cache, m, || {
                ExactBvh::build(
                    &STRICT,
                    &soup_polygons(&sphere(segments_for(m), [0.0; 3], 1.0)),
                )
                .expect("valid polygons")
                .into_value()
            });
            let bounds = ApproxBounds {
                min: point(0.6, -0.1, 0.5),
                max: point(0.8, 0.1, 0.7),
            };
            (bvh, bounds)
        },
        |(bvh, bounds)| {
            let mut hits = 0u64;
            bvh.query_bounds(&STRICT, bounds, |_| hits += 1)
                .map(|_| hits)
        },
    );
    atlas.operation(
        "broad phase",
        "point bvh halfspace",
        "points",
        "Build an exact point BVH and report the points on the positive side of an oblique plane through the cloud.",
    );
    atlas.sweep(
        "on sphere (dyadic)",
        &powers_of_two(4, 15),
        |n, rng| hull_points("on sphere (dyadic)", n as usize, rng),
        |points| {
            let plane = Plane::from_coefficients(r(1), r(2), r(3), ratio(-1, 7));
            ExactPointBvh::build(&STRICT, points).and_then(|bvh| {
                let mut hits = 0u64;
                bvh.into_value()
                    .query_positive_halfspace(&STRICT, points, &plane, |_| hits += 1)
                    .map(|_| hits)
            })
        },
    );

    atlas.operation(
        "query",
        "contains point",
        "triangles",
        "Exact point-in-solid against a fresh sphere (validation included): a generic interior point, and a point 2^-40 inside the surface.",
    );
    for kind in ["interior", "2^-40 below the surface"] {
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            &spheres,
            |m, _| {
                let mesh = memo(&mut cache, m, || sphere(segments_for(m), [0.0; 3], 1.0));
                let probe = if kind == "interior" {
                    point(0.1, 0.2, 0.3)
                } else {
                    // The top pole minus 2^-40 lies inside, near a vertex.
                    let p = &mesh.positions[0];
                    Point3::new(p.x.clone(), p.y.clone(), &p.z - &pow2_inverse(40))
                };
                (fresh(&mesh), probe)
            },
            |(mesh, probe)| {
                mesh.contains_point(&STRICT, probe)
                    .map(MeshOutcome::into_value)
            },
        );
    }
    atlas.operation(
        "query",
        "ray intersections",
        "triangles",
        "All exact ray hits from the centre: a generic direction through a sphere, and the box diagonal through grid vertices (degenerate hits).",
    );
    for kind in ["sphere, generic ray", "box diagonal through vertices"] {
        let sizes = if kind.starts_with("sphere") {
            spheres.clone()
        } else {
            box_sizes(6)
        };
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let mesh = memo(&mut cache, m, || {
                    if kind.starts_with("sphere") {
                        sphere(segments_for(m), [0.0; 3], 1.0)
                    } else {
                        boxed(box_level(m), [r(0), r(0), r(0)], &r(1))
                    }
                });
                let direction = if kind.starts_with("sphere") {
                    Vector3::from_xyz(r(3), r(5), r(7))
                } else {
                    Vector3::from_xyz(r(1), r(1), r(1))
                };
                (fresh(&mesh), direction)
            },
            |(mesh, direction)| {
                mesh.ray_intersections(&STRICT, &Point3::new(r(0), r(0), r(0)), direction)
                    .map(|hits| hits.into_value().len())
            },
        );
    }
    atlas.operation(
        "query",
        "polyline intersections",
        "polyline segments",
        "Intersections of a zig-zag tool path crossing a 960-triangle sphere.",
    );
    let body = sphere(32, [0.0; 3], 1.0);
    atlas.sweep(
        "zig-zag",
        &powers_of_two(0, 10),
        |n, _| {
            let n = n as usize;
            let path: Vec<Point3> = (0..=n)
                .map(|i| {
                    let x = -1.2 + 2.4 * i as f64 / n as f64;
                    point(x, 0.1, if i % 2 == 0 { -1.5 } else { 1.5 })
                })
                .collect();
            (fresh(&body), path)
        },
        |(mesh, path)| {
            mesh.polyline_intersections(&STRICT, path)
                .map(|hits| hits.into_value().len())
        },
    );

    atlas.operation(
        "validation",
        "validate closed pwn",
        "triangles",
        "Certify a fresh mesh as closed piecewise-winding-number input, as every Boolean does first.",
    );
    for kind in ["sphere (dyadic)", "box (coplanar grid)"] {
        let sizes = if kind.starts_with("sphere") {
            spheres.clone()
        } else {
            box_sizes(6)
        };
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                fresh(&memo(&mut cache, m, || {
                    if kind.starts_with("sphere") {
                        sphere(segments_for(m), [0.0; 3], 1.0)
                    } else {
                        boxed(box_level(m), [r(0), r(0), r(0)], &r(1))
                    }
                }))
            },
            |mesh| {
                mesh.validate_closed_pwn(&STRICT)
                    .map(MeshOutcome::into_value)
            },
        );
    }
    type Check = fn(&TriangleMesh) -> HypermeshResult<bool>;
    let checks: [(&str, &str, Check); 4] = [
        (
            "closed manifold geometry",
            "Exact closed-manifold geometry check of a fresh sphere.",
            |m| {
                m.is_closed_manifold_geometry(&STRICT)
                    .map(MeshOutcome::into_value)
            },
        ),
        (
            "unique nondegenerate triangles",
            "Exact duplicate and degeneracy scan of a fresh sphere.",
            |m| {
                m.has_unique_nondegenerate_triangles(&STRICT)
                    .map(MeshOutcome::into_value)
            },
        ),
        (
            "certify convex",
            "Certify that every vertex lies in every face half-space: binary64 UV spheres (not exactly convex, so they fail) and exact hulls of n sphere points (axis: triangles, or points for hulls).",
            |m| certify_convex_mesh(&STRICT, m.as_ref()).map(|_| true),
        ),
        (
            "exact bounds",
            "Exact axis-aligned bounds of a fresh sphere.",
            |m| m.exact_bounds(&STRICT).map(|b| b.into_value().is_some()),
        ),
    ];
    for (name, description, check) in checks {
        atlas.operation("validation", name, "triangles", description);
        let mut cache = HashMap::new();
        atlas.sweep(
            "sphere",
            &spheres,
            |m, _| {
                fresh(&memo(&mut cache, m, || {
                    sphere(segments_for(m), [0.0; 3], 1.0)
                }))
            },
            check,
        );
        if name == "certify convex" {
            let mut cache = HashMap::new();
            atlas.sweep(
                "exact hull of sphere points",
                &powers_of_two(4, 12),
                |n, _| {
                    fresh(&memo(&mut cache, n, || {
                        convex_ball(n as usize, [0.0; 3], 1.0)
                    }))
                },
                check,
            );
        }
    }
    atlas.operation(
        "validation",
        "adjacency",
        "triangles",
        "Combinatorial vertex adjacency of a sphere.",
    );
    atlas.sweep(
        "sphere",
        &spheres,
        |m, _| sphere(segments_for(m), [0.0; 3], 1.0),
        |mesh| mesh.adjacency().map(|a| a.len()),
    );
}

fn output(atlas: &mut Atlas) {
    let spheres: Vec<f64> = sphere_sizes()[..8].to_vec();
    atlas.operation(
        "output",
        "batch to meshes",
        "triangles per operand",
        "Convert a sphere-union Boolean batch to standalone triangle meshes and check closure.",
    );
    let mut cache = HashMap::new();
    atlas.sweep(
        "sphere union",
        &spheres,
        |m, _| {
            memo(&mut cache, m, || {
                let [a, b] = operands("spheres (dyadic)", m);
                pair(&STRICT, &a, &b, BooleanOp::Union).expect("sphere union")
            })
        },
        |batch| {
            let closed = batch.results.iter().all(boolean_mesh_is_closed);
            shaped_mesh(
                batch
                    .clone()
                    .into_triangle_meshes()
                    .and_then(|mut meshes| {
                        meshes.pop().ok_or(hypermesh::HypermeshError::EmptyInput)
                    })
                    .and_then(|mesh| {
                        if closed {
                            Ok(mesh)
                        } else {
                            Err(hypermesh::HypermeshError::EmptyInput)
                        }
                    }),
            )
        },
    );
    atlas.operation(
        "output",
        "gpu buffers",
        "triangles",
        "Exact flat-shaded render rows, then finite f32 GPU buffers, for dyadic spheres, rotated (irrational) spheres and smoothed (long-rational) spheres.",
    );
    for kind in ["dyadic", "rotated (irrational)", "smoothed (rational)"] {
        let mut cache = HashMap::new();
        atlas.sweep(
            kind,
            &spheres,
            |m, _| {
                memo(&mut cache, m, || {
                    let base = sphere(segments_for(m), [0.0; 3], 1.0);
                    match kind {
                        "dyadic" => base,
                        "rotated (irrational)" => rotated(&base, 30),
                        _ => base
                            .laplacian_smooth(&ratio(1, 3), 2)
                            .expect("smoothing a valid sphere"),
                    }
                })
            },
            |mesh| {
                mesh.to_exact_gpu_mesh_buffers()
                    .and_then(|rows| rows.try_approximate_f32())
                    .map(|buffers| buffers.indices.len())
            },
        );
    }
}

#[cfg(feature = "dispatch-trace")]
fn dispatch_events() -> u64 {
    hyperreal::dispatch_trace::snapshot()
        .iter()
        .map(|count| count.count)
        .sum()
}

fn main() {
    let mut atlas = Atlas::from_args("hypermesh");
    #[cfg(feature = "dispatch-trace")]
    let _recording = atlas.config().counters.then(|| {
        atlas.counter("dispatch events", dispatch_events);
        hyperreal::dispatch_trace::recording_scope()
    });
    booleans(&mut atlas);
    mesh_operations(&mut atlas);
    queries(&mut atlas);
    output(&mut atlas);
    atlas.finish();
}
