//! Performance atlas for Hypersdf: exact implicit-field evaluation, bounds and
//! meshing previews across CSG tree shape, scalar representation, distance to
//! the surface and grid resolution.
//!
//! Groups and axes:
//! - `evaluation`: point evaluation, classification and gradients, and `Sdf`
//!   construction, over CSG primitive count. Trees are left-deep union
//!   chains (depth n), balanced union trees (depth log n), alternating
//!   union/intersection chains, and chains of nested translations of one
//!   sphere. Point classification is also swept over the distance `2^-k` to
//!   the surface.
//! - `bounds`: conservative cell classification, interval and Lipschitz
//!   bounds over primitive count and over cell width `2^-k` next to the
//!   surface; `SdfProgram` native enclosures over subdivision depth.
//! - `meshing`: preview sampling, Surface Nets, dual and gradient contouring
//!   and exact voxel-cell classification over grid resolution (cells per
//!   axis), and native proof-bearing Hypervoxel enclosures over octree depth.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypersdf --all-features --example perf_atlas -- --out atlas.json`.
//! With `--resources`, evaluation results report the CSG tree's node count
//! and depth (`nodes`, `depth`, from the public `SdfExpr` variants), meshing
//! results their sample or vertex count, and subdivisions their cell count and
//! depth. With `--counters` and `dispatch-trace`, Hyperreal/Hyperlimit
//! dispatch, predicate and approximation events are recorded per call.
use std::sync::Arc;

use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlimit::Point3;
use hyperreal::{Rational, Real};
use hypersdf::{
    Sdf, SdfCellLocation, SdfExpr, SdfPointLocation, SdfPreviewGrid, SdfProgram,
    SdfSamplingPrecision, SdfSubdivisionOptions, SdfVoxelCellGrid, SdfVoxelEnclosureOptions,
};
use hypersolve::expression_program::ExpressionBudget;
use hypervoxel::{GridFrame, LengthUnit};
use perf_atlas::{Atlas, Outcome, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

fn int(v: i64) -> Real {
    Real::from(v)
}

fn ratio(n: i64, d: i64) -> Real {
    Real::new(Rational::new(n) / Rational::new(d))
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    p(int(x), int(y), int(z))
}

/// `2^-k` as an exact rational, for any `k >= 0`.
fn half_pow(k: u32) -> Real {
    let mut denominator = Rational::new(1);
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        denominator *= Rational::new(1 << step);
        left -= step;
    }
    Real::new(Rational::new(1) / denominator)
}

/// Node count and depth of an expression tree, from its public variants.
fn tree_shape(expr: &SdfExpr) -> Shape {
    let children: Vec<&SdfExpr> = match expr {
        SdfExpr::Union(a, b)
        | SdfExpr::Intersection(a, b)
        | SdfExpr::Add(a, b)
        | SdfExpr::Sub(a, b)
        | SdfExpr::Mul(a, b) => vec![a, b],
        SdfExpr::Abs(a)
        | SdfExpr::Sqrt(a)
        | SdfExpr::Sin(a)
        | SdfExpr::Cos(a)
        | SdfExpr::Tan(a)
        | SdfExpr::Complement(a)
        | SdfExpr::Offset { child: a, .. }
        | SdfExpr::Transform { child: a, .. } => vec![a],
        _ => Vec::new(),
    };
    let mut shape = Shape {
        nodes: 1,
        depth: 1,
        bits: 0,
    };
    for child in children {
        let c = tree_shape(child);
        shape.nodes += c.nodes;
        shape.depth = shape.depth.max(c.depth + 1);
    }
    shape
}

/// A field plus the shape of its tree, measured once outside the timer.
struct Field {
    sdf: Sdf,
    shape: Shape,
}

impl Field {
    fn new(expr: SdfExpr) -> Self {
        let shape = tree_shape(&expr);
        Self {
            sdf: Sdf::new(expr),
            shape,
        }
    }
}

/// A result carrying its field's tree shape for resource atlases; the shape
/// was measured outside the timer and is only copied here.
struct WithTree<T> {
    value: T,
    shape: Shape,
}

impl<T: Outcome> Outcome for WithTree<T> {
    fn succeeded(&self) -> bool {
        self.value.succeeded()
    }
    fn reason(&self) -> Option<String> {
        self.value.reason()
    }
    fn shape(&self) -> Option<Shape> {
        Some(self.shape)
    }
}

fn with_tree<T: Outcome>(value: T, shape: Shape) -> WithTree<T> {
    WithTree { value, shape }
}

/// Sphere `i` of a row of unit-radius-squared spheres along x.
fn ball(i: i64, rng: &mut Rng) -> SdfExpr {
    let c = pi(2 * i, rng.range(-1, 1), rng.range(-1, 1));
    SdfExpr::sphere(c, int(1 + rng.range(0, 3)))
}

const TREE_KINDS: [&str; 4] = [
    "left-deep union chain",
    "balanced union tree",
    "union/intersection chain",
    "nested translations",
];

fn balanced(mut leaves: Vec<SdfExpr>) -> SdfExpr {
    while leaves.len() > 1 {
        let mut next = Vec::with_capacity(leaves.len().div_ceil(2));
        let mut it = leaves.into_iter();
        while let Some(a) = it.next() {
            next.push(match it.next() {
                Some(b) => a.union(b),
                None => a,
            });
        }
        leaves = next;
    }
    leaves.pop().expect("at least one leaf")
}

/// A CSG tree with `n` primitives (or `n` nested transforms) of the given kind.
fn tree(kind: &str, n: usize, rng: &mut Rng) -> SdfExpr {
    let n = n.max(1);
    match kind {
        "left-deep union chain" => {
            (1..n as i64).fold(ball(0, rng), |acc, i| acc.union(ball(i, rng)))
        }
        "balanced union tree" => balanced((0..n as i64).map(|i| ball(i, rng)).collect()),
        "union/intersection chain" => (1..n as i64).fold(ball(0, rng), |acc, i| {
            if i % 2 == 0 {
                acc.union(ball(i, rng))
            } else {
                // A slab wide enough to keep the chain non-empty.
                acc.intersection(SdfExpr::aabb(pi(-4, -8, -8), pi(2 * i + 4, 8, 8)))
            }
        }),
        "nested translations" => (0..n).fold(SdfExpr::sphere(pi(0, 0, 0), int(4)), |acc, _| {
            acc.translate(p(ratio(rng.range(-8, 8), 7), int(0), int(rng.range(-1, 1))))
        }),
        other => unreachable!("unknown tree kind {other}"),
    }
}

/// A query point inside the frame covered by the generated trees.
fn query(n: usize, rng: &mut Rng) -> Point3 {
    p(
        ratio(rng.range(0, 2 * n as i64 * 16), 16),
        ratio(rng.range(-16, 16), 16),
        ratio(rng.range(-16, 16), 16),
    )
}

/// A small query cell inside the frame covered by the generated trees.
fn query_cell(n: usize, rng: &mut Rng) -> (Point3, Point3) {
    let min = query(n, rng);
    let w = ratio(1, 4);
    let max = p(
        min.x.clone() + w.clone(),
        min.y.clone() + w.clone(),
        min.z.clone() + w,
    );
    (min, max)
}

const SURFACE_KINDS: [&str; 4] = [
    "sphere",
    "irrational sphere",
    "torus",
    "union of two spheres",
];

/// A field of the given kind with surface point `(s, 0, 0)` on the +x side.
fn surface_field(kind: &str) -> (SdfExpr, Real) {
    match kind {
        "sphere" => (SdfExpr::sphere(pi(0, 0, 0), int(25)), int(5)),
        // Radius sqrt(2) itself: the surface point is irrational.
        "irrational sphere" => (
            SdfExpr::sphere(pi(0, 0, 0), int(2)),
            int(2).sqrt().expect("sqrt(2)"),
        ),
        // Major radius 4, minor radius 1, around z: outer equator at x = 5.
        "torus" => (
            SdfExpr::torus(hypersdf::SdfCoordinate::Z, pi(0, 0, 0), int(16), int(1)),
            int(5),
        ),
        "union of two spheres" => (
            SdfExpr::sphere(pi(0, 0, 0), int(25)).union(SdfExpr::sphere(pi(-6, 0, 0), int(25))),
            int(5),
        ),
        other => unreachable!("unknown surface kind {other}"),
    }
}

const SHAPES: [&str; 4] = [
    "sphere",
    "torus",
    "8-sphere union",
    "rounded box minus sphere",
];

/// A field filling roughly the unit cube `[-1, 1]^3`.
fn mesh_field(kind: &str) -> SdfExpr {
    match kind {
        "sphere" => SdfExpr::sphere(pi(0, 0, 0), ratio(3, 4)),
        "torus" => SdfExpr::torus(
            hypersdf::SdfCoordinate::Z,
            pi(0, 0, 0),
            ratio(1, 4),
            ratio(1, 16),
        ),
        "8-sphere union" => balanced(
            (0..8)
                .map(|i| {
                    let c = |bit: i64| ratio(if i & bit == 0 { -2 } else { 2 }, 5);
                    SdfExpr::sphere(p(c(1), c(2), c(4)), ratio(1, 9))
                })
                .collect(),
        ),
        "rounded box minus sphere" => SdfExpr::rounded_aabb(
            p(ratio(-3, 5), ratio(-3, 5), ratio(-3, 5)),
            p(ratio(3, 5), ratio(3, 5), ratio(3, 5)),
            ratio(1, 100),
        )
        .intersection(SdfExpr::sphere(pi(0, 0, 0), ratio(1, 2)).complement()),
        other => unreachable!("unknown mesh kind {other}"),
    }
}

/// `[-1, 1]^3` split into `n` cells per axis.
fn preview_grid(n: u32) -> SdfPreviewGrid {
    let step = ratio(2, i64::from(n));
    SdfPreviewGrid::new(
        pi(-1, -1, -1),
        p(step.clone(), step.clone(), step),
        [n + 1; 3],
    )
}

fn voxel_grid(n: u32) -> SdfVoxelCellGrid {
    let step = ratio(2, i64::from(n));
    SdfVoxelCellGrid::new(pi(-1, -1, -1), p(step.clone(), step.clone(), step), [n; 3])
}

fn graph_budget() -> ExpressionBudget {
    ExpressionBudget::new(1 << 30, usize::MAX / 2)
}

fn arithmetic_budget() -> DyadicBudget {
    DyadicBudget::new(1 << 16, u64::MAX / 2)
}

/// Fields the native enclosure program accepts (polynomial primitives and
/// smooth arithmetic, no piecewise CSG).
const PROGRAM_KINDS: [&str; 3] = ["sphere", "torus", "sine sheet"];

fn program_field(kind: &str) -> SdfExpr {
    match kind {
        "sphere" | "torus" => mesh_field(kind),
        // z - sin(4x)/4: a smooth non-polynomial surface.
        "sine sheet" => SdfExpr::z().sub_expr(
            SdfExpr::x()
                .mul_expr(SdfExpr::constant(int(4)))
                .sin()
                .mul_expr(SdfExpr::constant(ratio(1, 4))),
        ),
        other => unreachable!("unknown program kind {other}"),
    }
}

#[cfg(feature = "dispatch-trace")]
fn correlation() -> hyperreal::dispatch_trace::TraceCorrelationSummary {
    hyperreal::dispatch_trace::snapshot_trace().correlation_summary()
}

#[cfg(feature = "dispatch-trace")]
fn dispatch_events() -> u64 {
    correlation().dispatch_events
}

#[cfg(feature = "dispatch-trace")]
fn predicate_events() -> u64 {
    correlation().predicate_events
}

#[cfg(feature = "dispatch-trace")]
fn approximation_events() -> u64 {
    correlation().approximation_events
}

/// Stack for the atlas thread. Field queries recurse once per tree level
/// (about 1.4 KiB per level), so deep CSG chains overflow the 8 MiB main
/// thread; timing runs therefore use a large dedicated stack, and resource
/// runs report the measured depth.
const ATLAS_STACK: usize = 1 << 30;

fn main() {
    std::thread::Builder::new()
        .name("perf-atlas".to_owned())
        .stack_size(ATLAS_STACK)
        .spawn(run)
        .expect("atlas thread")
        .join()
        .expect("atlas run");
}

fn run() {
    let mut atlas = Atlas::from_args("hypersdf");
    // Recording dispatch labels allocates, so it runs only when asked for.
    #[cfg(feature = "dispatch-trace")]
    let _recording = atlas.config().counters.then(|| {
        atlas.counter("dispatch events", dispatch_events);
        atlas.counter("predicate events", predicate_events);
        atlas.counter("approximation events", approximation_events);
        hyperreal::dispatch_trace::recording_scope()
    });
    evaluation(&mut atlas);
    bounds(&mut atlas);
    meshing(&mut atlas);
    atlas.finish();
}

fn evaluation(atlas: &mut Atlas) {
    let sizes = powers_of_two(0, 13);
    atlas.operation(
        "evaluation",
        "evaluate point",
        "CSG primitive count",
        "Exact field value at a rational point, approximated to f64; sphere distances carry square roots.",
    );
    for kind in TREE_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                (
                    Field::new(tree(kind, n as usize, rng)),
                    query(n as usize, rng),
                )
            },
            |(field, point)| {
                let value = field
                    .sdf
                    .evaluate_point(point)
                    .ok()
                    .and_then(|v| v.to_f64_lossy());
                with_tree(value, field.shape)
            },
        );
    }

    atlas.operation(
        "evaluation",
        "classify point",
        "CSG primitive count",
        "Exact inside/boundary/outside classification of a rational point.",
    );
    for kind in TREE_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                (
                    Field::new(tree(kind, n as usize, rng)),
                    query(n as usize, rng),
                )
            },
            |(field, point)| {
                let located = field.sdf.classify_point(point).location != SdfPointLocation::Unknown;
                with_tree(located, field.shape)
            },
        );
    }

    atlas.operation(
        "evaluation",
        "gradient",
        "CSG primitive count",
        "Exact field gradient at a rational point (None at nonsmooth CSG ties).",
    );
    for kind in TREE_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                (
                    Field::new(tree(kind, n as usize, rng)),
                    query(n as usize, rng),
                )
            },
            |(field, point)| {
                let gradient = field.sdf.gradient_point(point).gradient.is_some();
                with_tree(gradient, field.shape)
            },
        );
    }

    atlas.operation(
        "evaluation",
        "build field facts",
        "CSG primitive count",
        "Sdf::new: one pass of structural facts (metric, domain, gradient, Lipschitz status) over a cloned tree.",
    );
    for kind in TREE_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| Field::new(tree(kind, n as usize, rng)),
            |field| {
                let facts = Sdf::new(field.sdf.expr().clone()).facts().node_count;
                with_tree(facts, field.shape)
            },
        );
    }

    let exponents = powers_of_two(0, 12);
    atlas.operation(
        "evaluation",
        "classify point near surface",
        "distance exponent k (point 2^-k outside the surface)",
        "Classify a point just outside the surface: distance to the boundary decision.",
    );
    for kind in SURFACE_KINDS {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| {
                let (expr, s) = surface_field(kind);
                (Sdf::new(expr), p(s + half_pow(k as u32), int(0), int(0)))
            },
            |(sdf, point)| sdf.classify_point(point).location == SdfPointLocation::Outside,
        );
    }
}

fn bounds(atlas: &mut Atlas) {
    let sizes = powers_of_two(0, 13);
    for (name, description) in [
        (
            "classify cell",
            "Conservative inside/boundary/outside classification of a 1/4-wide cell.",
        ),
        (
            "interval bound",
            "Exact interval enclosure of the field over a 1/4-wide cell.",
        ),
        (
            "Lipschitz bound",
            "Certified Lipschitz bound of the field over a 1/4-wide cell.",
        ),
    ] {
        atlas.operation("bounds", name, "CSG primitive count", description);
        for kind in TREE_KINDS {
            atlas.sweep(
                kind,
                &sizes,
                |n, rng| {
                    (
                        Field::new(tree(kind, n as usize, rng)),
                        query_cell(n as usize, rng),
                    )
                },
                |(field, (min, max))| {
                    let decided = match name {
                        "classify cell" => {
                            field.sdf.classify_cell(min, max).location != SdfCellLocation::Unknown
                        }
                        "interval bound" => field.sdf.interval_cell(min, max).interval.is_some(),
                        _ => field.sdf.lipschitz_cell(min, max).bound.is_some(),
                    };
                    with_tree(decided, field.shape)
                },
            );
        }
    }

    atlas.operation(
        "bounds",
        "classify cell near surface",
        "width exponent k (cell [s + 2^-k, s + 2^(1-k)] beyond surface point s)",
        "Conservative classification of a shrinking cell just outside the surface; succeeds only when it certifies outside.",
    );
    for kind in SURFACE_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |k, _| {
                let (expr, s) = surface_field(kind);
                let w = half_pow(k as u32);
                let lo = s + w.clone();
                let hi = lo.clone() + w.clone();
                (
                    Sdf::new(expr),
                    p(lo, -w.clone(), -w.clone()),
                    p(hi, w.clone(), w),
                )
            },
            |(sdf, min, max)| match sdf.classify_cell(min, max).location {
                SdfCellLocation::ConservativeOutside => Ok(()),
                other => Err(format!("{other:?}")),
            },
        );
    }

    atlas.operation(
        "bounds",
        "program subdivision",
        "maximum subdivision depth",
        "SdfProgram::subdivide: adaptive native dyadic enclosure cover of [-1, 1]^3 down to the given depth.",
    );
    for kind in PROGRAM_KINDS {
        atlas.sweep(
            kind,
            &(1..=18).map(f64::from).collect::<Vec<_>>(),
            |depth, _| {
                let program = SdfProgram::compile(
                    Arc::new(Sdf::new(program_field(kind))),
                    &mut graph_budget(),
                )
                .expect("supported field");
                (program, depth as u32)
            },
            |(program, depth)| {
                let unit = || DyadicInterval::new((-1).into(), 1.into(), 0).expect("[-1, 1]");
                let cover = program.subdivide(
                    [unit(), unit(), unit()],
                    SdfSubdivisionOptions {
                        precision: -48,
                        max_depth: *depth,
                        max_cells: 1 << 24,
                    },
                    &mut graph_budget(),
                    &mut arithmetic_budget(),
                );
                Shaped::new(cover, |c| {
                    c.as_ref().ok().map(|c| Shape {
                        nodes: c.cells().len() as u64,
                        depth: c
                            .cells()
                            .iter()
                            .map(|cell| u64::from(cell.depth()))
                            .max()
                            .unwrap_or(0),
                        bits: 0,
                    })
                })
            },
        );
    }
}

fn meshing(atlas: &mut Atlas) {
    let resolutions = [
        2.0, 4.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0, 96.0, 128.0,
    ];
    atlas.operation(
        "meshing",
        "sample grid preview",
        "cells per axis",
        "Lossy f64 field samples on an (n+1)^3 preview grid over [-1, 1]^3.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &resolutions,
            |n, _| (Sdf::new(mesh_field(kind)), preview_grid(n as u32)),
            |(sdf, grid)| {
                Shaped::new(
                    sdf.sample_grid_preview(grid.clone(), SdfSamplingPrecision::F64),
                    |r| {
                        r.as_ref().ok().map(|r| Shape {
                            nodes: r.samples.sample_count as u64,
                            ..Shape::default()
                        })
                    },
                )
            },
        );
    }

    atlas.operation(
        "meshing",
        "surface nets preview",
        "cells per axis",
        "Sample the preview grid, then run Surface Nets diagnostics (crossing edges and proposed vertices).",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &resolutions,
            |n, _| (Sdf::new(mesh_field(kind)), preview_grid(n as u32)),
            |(sdf, grid)| {
                Shaped::new(
                    sdf.mesh_preview_from_grid(grid.clone(), SdfSamplingPrecision::F64),
                    |r| {
                        r.as_ref().ok().map(|r| Shape {
                            nodes: r.vertex_count as u64,
                            ..Shape::default()
                        })
                    },
                )
            },
        );
    }

    atlas.operation(
        "meshing",
        "dual contouring report",
        "cells per axis",
        "Sample the preview grid, then retain dual-contouring crossings, exact edge roots and vertex proposals.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &resolutions[..8],
            |n, _| (Sdf::new(mesh_field(kind)), preview_grid(n as u32)),
            |(sdf, grid)| {
                Shaped::new(
                    sdf.dual_contouring_report_from_grid(grid.clone(), SdfSamplingPrecision::F64),
                    |r| {
                        r.as_ref().ok().map(|r| Shape {
                            nodes: r.active_cell_count() as u64,
                            ..Shape::default()
                        })
                    },
                )
            },
        );
    }

    atlas.operation(
        "meshing",
        "gradient contouring report",
        "cells per axis",
        "Sample the preview grid, then retain gradient projections for every active cell.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &resolutions[..8],
            |n, _| (Sdf::new(mesh_field(kind)), preview_grid(n as u32)),
            |(sdf, grid)| {
                Shaped::new(
                    sdf.gradient_contouring_report_from_grid(
                        grid.clone(),
                        SdfSamplingPrecision::F64,
                    ),
                    |r| {
                        r.as_ref().ok().map(|r| Shape {
                            nodes: r.active_cell_count() as u64,
                            ..Shape::default()
                        })
                    },
                )
            },
        );
    }

    atlas.operation(
        "meshing",
        "exact voxel classification",
        "cells per axis",
        "Sdf::classify_voxel_grid: conservative exact classification of all n^3 cells; unknown cells mark failure.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &resolutions[..9],
            |n, _| (Sdf::new(mesh_field(kind)), voxel_grid(n as u32)),
            |(sdf, grid)| {
                let batch = sdf
                    .classify_voxel_grid(grid.clone())
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|b| {
                        if b.has_unknown() {
                            Err("unknown cells".to_owned())
                        } else {
                            Ok(b)
                        }
                    });
                Shaped::new(batch, |b| {
                    b.as_ref().ok().map(|b| Shape {
                        nodes: b.cells.len() as u64,
                        ..Shape::default()
                    })
                })
            },
        );
    }

    atlas.operation(
        "meshing",
        "native voxel enclosure",
        "octree depth (2^depth cells per axis)",
        "SdfProgram::classify_voxel_grid: a native range proof for every finest cell of a Hypervoxel frame over [-1, 1]^3.",
    );
    for kind in PROGRAM_KINDS {
        atlas.sweep(
            kind,
            &(1..=7).map(f64::from).collect::<Vec<_>>(),
            |depth, _| {
                let program = SdfProgram::compile(
                    Arc::new(Sdf::new(program_field(kind))),
                    &mut graph_budget(),
                )
                .expect("supported field");
                let pitch = ratio(2, 1 << depth as i64);
                let frame = GridFrame::new(
                    [int(-1), int(-1), int(-1)],
                    [pitch.clone(), pitch.clone(), pitch],
                    depth as u8,
                    LengthUnit::Millimeter,
                )
                .expect("frame");
                (program, frame)
            },
            |(program, frame)| {
                let batch = program.classify_voxel_grid(
                    frame.clone(),
                    SdfVoxelEnclosureOptions {
                        precision: -48,
                        max_cells: 1 << 24,
                    },
                    &mut graph_budget(),
                    &mut arithmetic_budget(),
                );
                Shaped::new(batch, |b| {
                    b.as_ref().ok().map(|b| Shape {
                        nodes: b.cells().len() as u64,
                        depth: u64::from(b.frame().depth()),
                        bits: 0,
                    })
                })
            },
        );
    }
}
