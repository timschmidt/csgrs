//! Performance and resource atlas for csgrs: the CAD/CAM workflows built on
//! the native Hyper geometry crates.
//!
//! Covered, in workflow order:
//! - sketching: profile primitives (gears), 2D Booleans of independently
//!   sized regions, plates with growing hole counts, offsetting (round,
//!   miter, inward and outward) and triangulation;
//! - solid construction: extrude, twisted extrude, revolve, sweep along a
//!   helix, loft, and mesh primitives (sphere, cylinder, torus, spur gear);
//! - solid Booleans: an independently sized operand grid, and drilled plates
//!   evaluated eagerly or as one lazy `SolidExpr` program (linear or balanced);
//! - manufacturing queries: slicing (single slices and layer stacks into
//!   filled regions), XY flattening, exact mass properties, rounding by
//!   Minkowski sum, sampled SDF, gyroid infill and metaball solids, Hershey
//!   engraving text, and grid patterning;
//! - interchange: STL (binary and ASCII), OBJ, PLY, AMF, glTF and DXF export
//!   with output sizes, STL/OBJ/glTF import, and SVG/Gerber 2D round trips;
//! - scalar backends: the `adapter` facade over raw `Real`, `f32`, `f64` and
//!   `i128` scalars;
//! - grammar caches: the same primitive with the default cache and with
//!   caching disabled.
//!
//! Axes are segment, tooth, hole, layer, slice, sample or triangle counts as
//! labelled. Every operation except the cache probe runs with grammar caches
//! disabled, so repeated calls do real work; the cache probe shows what the
//! default policy retains. With `--resources`, meshes report `Shape { nodes:
//! triangles, depth: deepest coordinate expression, bits: largest stored
//! coordinate rational }`; regions report boundary fragments; lazy CSG
//! programs report their expression depth; exports report output bytes as
//! nodes.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p csgrs --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use std::collections::HashMap;
use std::f64::consts::TAU;

use csgrs::adapter::{F32, F64, I128, RawReal, ScalarAdapter, ScalarMesh};
use csgrs::curve::{self, hershey};
use csgrs::io::{amf, dxf, gerber, gltf, obj, ply, stl, svg};
use csgrs::solid::cache::{self, CachePolicy};
use csgrs::solid::{self, SolidExt};
use csgrs::{GeometryContext, Real, SolidExpr, TriangleMesh};
use hypercurve::{BooleanOp as RegionOp, CurveRegion2, OffsetCornerStyle2};
use hyperlattice::Point3;
use hypermesh::BooleanOp;
use perf_atlas::{Atlas, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const STRICT: GeometryContext = GeometryContext::STRICT;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn f(value: f64) -> Real {
    Real::try_from(value).expect("finite coordinate")
}

fn ratio(numerator: i64, denominator: i64) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero denominator")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(f(x), f(y), f(z))
}

/// Build an input once per magnitude, then hand out copies of it.
fn memo<V: Clone>(cache: &mut HashMap<u64, V>, key: f64, build: impl FnOnce() -> V) -> V {
    cache.entry(key.to_bits()).or_insert_with(build).clone()
}

/// A translated copy of a region: new storage, so no classification or
/// regularization cached by an earlier call is reused.
fn fresh_region(region: &CurveRegion2) -> CurveRegion2 {
    curve::translated(region, 1, 0).expect("finite translation")
}

fn coordinate_shape<'a>(points: impl IntoIterator<Item = &'a Point3>) -> (u64, u64) {
    points
        .into_iter()
        .flat_map(|p| [&p.x, &p.y, &p.z])
        .map(|c| c.representation_size(1 << 12))
        .fold((0, 0), |(depth, bits), size| {
            (depth.max(size.depth as u64), bits.max(size.max_rational_bits))
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

fn shaped_mesh<E>(result: Result<TriangleMesh, E>) -> Shaped<Result<TriangleMesh, E>>
where
    E: std::fmt::Debug,
{
    Shaped::new(result, |result| result.as_ref().ok().map(mesh_shape))
}

fn region_shape(region: &CurveRegion2) -> Shape {
    Shape {
        nodes: region.boundary_loops().iter().map(|l| l.len() as u64).sum(),
        depth: region.len() as u64,
        bits: 0,
    }
}

fn shaped_region<E>(result: Result<CurveRegion2, E>) -> Shaped<Result<CurveRegion2, E>>
where
    E: std::fmt::Debug,
{
    Shaped::new(result, |result| result.as_ref().ok().map(region_shape))
}

/// Serialized output, reporting its byte length as nodes.
fn shaped_bytes<E: std::fmt::Debug>(result: Result<usize, E>) -> Shaped<Result<usize, E>> {
    Shaped::new(result, |result| {
        result.as_ref().ok().map(|&bytes| Shape {
            nodes: bytes as u64,
            depth: 0,
            bits: 0,
        })
    })
}

/// Sphere sizes by longitude count; `solid::sphere(r, s, s / 2)`.
fn sphere(segments: usize) -> TriangleMesh {
    solid::sphere(r(1), segments, segments / 2).expect("valid sphere")
}

fn segments() -> Vec<f64> {
    powers_of_two(3, 8)
}

/// A rectangular plate with `holes` circular holes of `segments` sides.
fn plate_region(holes: usize, segments: usize) -> (CurveRegion2, Vec<CurveRegion2>) {
    let side = (holes as f64).sqrt().ceil().max(1.0) as usize;
    let pitch = 10.0;
    let width = pitch * side as f64;
    let plate = curve::rectangle(f(width), f(width)).expect("valid plate");
    let holes = (0..holes)
        .map(|i| {
            let x = pitch * ((i % side) as f64 + 0.5);
            let y = pitch * ((i / side) as f64 + 0.5);
            let circle = curve::circle(f(3.0), segments).expect("valid hole");
            curve::translated(&circle, f(x), f(y)).expect("valid hole")
        })
        .collect();
    (plate, holes)
}

fn region_boolean(
    a: &CurveRegion2,
    b: &CurveRegion2,
    op: RegionOp,
) -> Result<CurveRegion2, String> {
    a.boolean_region(b, op).map_err(|e| format!("{e:?}"))
}

fn sketching(atlas: &mut Atlas) {
    atlas.operation(
        "sketch",
        "involute gear profile",
        "teeth",
        "An involute spur-gear outline (module 1, 20 degree pressure angle, four segments per flank) assembled as one filled region.",
    );
    atlas.sweep(
        "spur gear",
        &[4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0],
        |teeth, _| teeth as usize,
        |&teeth| shaped_region(curve::involute_gear(r(1), teeth, r(20), ratio(1, 4), r(0), 4)),
    );

    let sizes = segments();
    atlas.operation(
        "sketch",
        "region union grid",
        "segments in A",
        "Exact union of two overlapping circle regions whose segment counts vary independently.",
    );
    atlas.grid(
        "circle ∪ circle",
        ("segments in A", &sizes),
        ("segments in B", &sizes),
        |a, b, _| {
            let left = curve::circle(r(1), a as usize).expect("valid circle");
            let right = curve::translated(
                &curve::circle(f(0.8), b as usize).expect("valid circle"),
                f(0.7),
                f(0.3),
            )
            .expect("valid translation");
            (left, right)
        },
        |(a, b)| shaped_region(region_boolean(a, b, RegionOp::Union)),
    );

    let holes = powers_of_two(0, 7);
    atlas.operation(
        "sketch",
        "plate with holes",
        "holes",
        "A rectangular plate minus k 16-sided holes: one difference per hole, or the holes unioned first and subtracted once.",
    );
    atlas.sweep(
        "one difference per hole",
        &holes,
        |k, _| plate_region(k as usize, 16),
        |(plate, holes)| {
            let mut acc = Ok(plate.clone());
            for hole in holes {
                acc = acc.and_then(|body| region_boolean(&body, hole, RegionOp::Difference));
            }
            shaped_region(acc)
        },
    );
    atlas.sweep(
        "union holes, then subtract",
        &holes,
        |k, _| plate_region(k as usize, 16),
        |(plate, holes)| {
            let mut tools = Ok(holes[0].clone());
            for hole in &holes[1..] {
                tools = tools.and_then(|acc| region_boolean(&acc, hole, RegionOp::Union));
            }
            shaped_region(
                tools.and_then(|tools| region_boolean(plate, &tools, RegionOp::Difference)),
            )
        },
    );

    atlas.operation(
        "sketch",
        "offset",
        "star points",
        "Exact offset of an n-pointed star (2n vertices, alternating convex and reflex corners) by round, miter and inward round corners.",
    );
    let points = [3.0, 4.0, 5.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0];
    for (kind, distance, style) in [
        ("outward round", ratio(1, 10), OffsetCornerStyle2::Round),
        (
            "outward miter",
            ratio(1, 10),
            OffsetCornerStyle2::Miter { limit: r(4) },
        ),
        ("inward round", ratio(-1, 20), OffsetCornerStyle2::Round),
    ] {
        atlas.sweep(
            kind,
            &points,
            |n, _| curve::star(n as usize, r(2), r(1)).expect("valid star"),
            |star| shaped_region(star.offset(distance.clone(), &style)),
        );
    }
    atlas.operation(
        "sketch",
        "offset gear",
        "teeth",
        "Round offset of an involute gear outline outward by a tenth of a module: a tool-radius compensation pass.",
    );
    atlas.sweep(
        "outward round",
        &[4.0, 5.0, 6.0, 8.0, 12.0, 16.0],
        |teeth, _| {
            curve::involute_gear(r(1), teeth as usize, r(20), ratio(1, 4), r(0), 4)
                .expect("valid gear")
        },
        |gear| shaped_region(gear.offset(ratio(1, 10), &OffsetCornerStyle2::Round)),
    );

    atlas.operation(
        "sketch",
        "triangulate",
        "holes",
        "Exact triangulation of a plate region with k 16-sided holes.",
    );
    let mut plates = HashMap::new();
    atlas.sweep(
        "plate with holes",
        &holes,
        |k, _| {
            fresh_region(&memo(&mut plates, k, || {
                let (plate, holes) = plate_region(k as usize, 16);
                let tools = holes
                    .iter()
                    .skip(1)
                    .try_fold(holes[0].clone(), |acc, hole| {
                        region_boolean(&acc, hole, RegionOp::Union)
                    })
                    .expect("disjoint holes");
                region_boolean(&plate, &tools, RegionOp::Difference)
                    .expect("plate minus holes")
            }))
        },
        |region| shaped_mesh(curve::try_triangulate(region, STRICT).map(|o| o.into_value())),
    );
}

fn construction(atlas: &mut Atlas) {
    atlas.operation(
        "construct",
        "extrude",
        "profile segments or teeth",
        "Linear extrusion of a regular polygon (segments), an involute gear (teeth) and a plate with holes (hole count).",
    );
    atlas.sweep(
        "regular polygon",
        &powers_of_two(3, 10),
        |n, _| curve::regular_ngon(n as usize, r(1)).expect("valid polygon"),
        |region| shaped_mesh(curve::try_extrude(region, r(2), STRICT).map(|o| o.into_value())),
    );
    atlas.sweep(
        "involute gear",
        &[4.0, 6.0, 8.0, 12.0, 16.0, 32.0],
        |teeth, _| {
            curve::involute_gear(r(1), teeth as usize, r(20), ratio(1, 4), r(0), 4)
                .expect("valid gear")
        },
        |region| shaped_mesh(curve::try_extrude(region, r(5), STRICT).map(|o| o.into_value())),
    );
    let mut plates = HashMap::new();
    atlas.sweep(
        "plate with holes",
        &powers_of_two(0, 7),
        |k, _| {
            fresh_region(&memo(&mut plates, k, || {
                let (plate, holes) = plate_region(k as usize, 16);
                holes
                    .iter()
                    .try_fold(plate, |acc, hole| {
                        region_boolean(&acc, hole, RegionOp::Difference)
                    })
                    .expect("plate minus holes")
            }))
        },
        |region| shaped_mesh(curve::try_extrude(region, r(3), STRICT).map(|o| o.into_value())),
    );

    atlas.operation(
        "construct",
        "extrude twisted",
        "slices",
        "Twisted, tapered extrusion (90 degrees, end scale 1/2) of a square into growing slice counts.",
    );
    let square = curve::square(r(2)).expect("valid square");
    atlas.sweep(
        "square",
        &powers_of_two(1, 9),
        |n, _| n as usize,
        |&slices| {
            shaped_mesh(
                curve::extrude_twisted(
                    &square,
                    r(4),
                    r(90),
                    [ratio(1, 2), ratio(1, 2)],
                    slices,
                    STRICT,
                )
                .map(|o| o.into_value()),
            )
        },
    );

    atlas.operation(
        "construct",
        "revolve",
        "revolution segments",
        "Revolve a 16-sided circle profile offset from the axis: a full turn (torus) and a quarter turn (capped).",
    );
    let profile = curve::translated(
        &curve::circle(ratio(1, 2), 16).expect("valid circle"),
        r(2),
        r(0),
    )
    .expect("valid profile");
    for (kind, degrees) in [("360 degrees", 360), ("90 degrees", 90)] {
        atlas.sweep(
            kind,
            &powers_of_two(3, 10),
            |n, _| n as usize,
            |&segments| {
                shaped_mesh(
                    curve::revolve(&profile, r(degrees), segments, STRICT)
                        .map(|o| o.into_value()),
                )
            },
        );
    }

    atlas.operation(
        "construct",
        "sweep",
        "path points",
        "Sweep a 12-sided circle along a helix with growing point counts (a coil or thread).",
    );
    let wire = curve::circle(ratio(1, 10), 12).expect("valid circle");
    atlas.sweep(
        "helix",
        &[2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 64.0],
        |n, _| {
            (0..n as usize)
                .map(|i| {
                    let t = i as f64 * 0.15;
                    p3(t.cos(), t.sin(), i as f64 * 0.04)
                })
                .collect::<Vec<_>>()
        },
        |path| shaped_mesh(curve::try_sweep(&wire, path, STRICT).map(|o| o.into_value())),
    );

    atlas.operation(
        "construct",
        "loft",
        "sections",
        "Loft through k 32-point rings whose radius varies with height.",
    );
    atlas.sweep(
        "rings",
        &powers_of_two(1, 9),
        |k, _| {
            (0..k as usize)
                .map(|level| {
                    let radius = 1.0 + 0.3 * (level as f64 * 0.7).sin();
                    (0..32)
                        .map(|j| {
                            let theta = TAU * j as f64 / 32.0;
                            p3(radius * theta.cos(), radius * theta.sin(), level as f64)
                        })
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>()
        },
        |sections| shaped_mesh(solid::loft(sections)),
    );

    atlas.operation(
        "construct",
        "primitives",
        "segments",
        "Mesh primitives by segment count: sphere (segments x segments/2), cylinder, and torus (segments x segments/2).",
    );
    type Primitive = fn(usize) -> Result<TriangleMesh, csgrs::errors::ValidationError>;
    let primitives: [(&str, Primitive); 3] = [
        ("sphere", |s| solid::sphere(r(1), s, s / 2)),
        ("cylinder", |s| solid::cylinder(r(1), r(2), s)),
        ("torus", |s| solid::torus(r(2), ratio(1, 2), s, s / 2)),
    ];
    for (kind, build) in primitives {
        atlas.sweep(
            kind,
            &powers_of_two(3, 9),
            |s, _| s as usize,
            move |&s| shaped_mesh(build(s)),
        );
    }
    atlas.operation(
        "construct",
        "spur gear solid",
        "teeth",
        "Extruded involute spur gear solid (module 1, four segments per flank, thickness 5).",
    );
    atlas.sweep(
        "spur gear",
        &[4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0],
        |teeth, _| teeth as usize,
        |&teeth| {
            shaped_mesh(
                solid::spur_gear_involute(
                    r(1),
                    teeth,
                    r(20),
                    ratio(1, 4),
                    r(0),
                    4,
                    r(5),
                    STRICT,
                )
                .map(|o| o.into_value()),
            )
        },
    );
}

/// A 2-unit plate and `holes` tools on a grid: 16-sided through-cylinders
/// (exact trigonometric coordinates) or square through-pockets (integers).
fn drilled_plate(holes: usize, round: bool) -> (TriangleMesh, Vec<TriangleMesh>) {
    let side = (holes as f64).sqrt().ceil().max(1.0) as usize;
    let pitch = 4;
    let width = (pitch * side) as i64;
    let plate = solid::cuboid(r(width), r(width), r(2)).expect("valid plate");
    let tools = (0..holes)
        .map(|i| {
            let x = pitch as f64 * ((i % side) as f64 + 0.5);
            let y = pitch as f64 * ((i / side) as f64 + 0.5);
            let tool = if round {
                solid::cylinder(r(1), r(4), 16)
            } else {
                solid::cuboid(r(2), r(2), r(4))
            };
            tool.and_then(|c| {
                c.translated(
                    f(x - if round { 0.0 } else { 1.0 }),
                    f(y - if round { 0.0 } else { 1.0 }),
                    r(-1),
                )
            })
            .expect("valid tool")
        })
        .collect();
    (plate, tools)
}

fn expr_depth(nodes: usize, balanced: bool) -> u64 {
    if balanced {
        (nodes as f64).log2().ceil() as u64 + 2
    } else {
        nodes as u64 + 1
    }
}

fn solid_booleans(atlas: &mut Atlas) {
    let sizes = vec![4.0, 6.0, 8.0, 12.0, 16.0, 24.0];
    atlas.operation(
        "solid boolean",
        "union grid",
        "sphere segments",
        "Exact union of a sphere (segments x segments/2) and a crossing cylinder whose segment counts vary independently.",
    );
    atlas.grid(
        "sphere ∪ cylinder",
        ("sphere segments", &sizes),
        ("cylinder segments", &sizes),
        |a, b, _| {
            let cylinder = solid::cylinder(ratio(1, 2), r(3), b as usize)
                .and_then(|c| c.translated(f(0.3), f(0.2), f(-1.5)))
                .expect("valid cylinder");
            (sphere(a as usize), cylinder)
        },
        |(a, b)| shaped_mesh(solid::boolean(a, b, BooleanOp::Union)),
    );
    atlas.operation(
        "solid boolean",
        "by operation",
        "sphere segments",
        "Each Boolean between a sphere and a crossing cylinder of equal segment counts.",
    );
    for (kind, op) in [
        ("union", BooleanOp::Union),
        ("intersection", BooleanOp::Intersection),
        ("difference", BooleanOp::Difference),
        ("symmetric difference", BooleanOp::SymmetricDifference),
    ] {
        atlas.sweep(
            kind,
            &sizes,
            |s, _| {
                let cylinder = solid::cylinder(ratio(1, 2), r(3), s as usize)
                    .and_then(|c| c.translated(f(0.3), f(0.2), f(-1.5)))
                    .expect("valid cylinder");
                (sphere(s as usize), cylinder)
            },
            move |(a, b)| shaped_mesh(solid::boolean(a, b, op)),
        );
    }

    let holes = powers_of_two(0, 6);
    atlas.operation(
        "solid boolean",
        "drilled plate",
        "holes",
        "A plate minus k square through-pockets: eager differences (one arrangement per pocket), or one lazy SolidExpr program as a linear chain or a balanced union of tools; and eager differences of 16-sided cylinders, whose exact trigonometric coordinates cost far more. Shape depth is the CSG depth.",
    );
    for (kind, round) in [
        ("eager differences", false),
        ("eager, 16-sided cylinders", true),
    ] {
        atlas.sweep(
            kind,
            &holes,
            |k, _| drilled_plate(k as usize, round),
            |(plate, tools)| {
                let mut acc = Ok(plate.clone());
                for tool in tools {
                    acc = acc.and_then(|body| body.try_difference(tool));
                }
                let depth = tools.len() as u64 + 1;
                Shaped::new((acc, depth), |(acc, depth)| {
                    acc.as_ref().ok().map(|mesh| Shape {
                        depth: *depth,
                        ..mesh_shape(mesh)
                    })
                })
            },
        );
    }
    for (kind, balanced) in [
        ("SolidExpr linear chain", false),
        ("SolidExpr balanced", true),
    ] {
        atlas.sweep(
            kind,
            &holes,
            |k, _| {
                let (plate, tools) = drilled_plate(k as usize, false);
                let tools: Vec<SolidExpr> = tools.into_iter().map(SolidExpr::from).collect();
                let expression = if balanced {
                    let mut level = tools;
                    while level.len() > 1 {
                        level = level
                            .chunks(2)
                            .map(|pair| match pair {
                                [a, b] => a.union(b),
                                [a] => a.clone(),
                                _ => unreachable!("chunks of two"),
                            })
                            .collect();
                    }
                    SolidExpr::from(plate).difference(&level[0])
                } else {
                    tools
                        .iter()
                        .fold(SolidExpr::from(plate), |acc, tool| acc.difference(tool))
                };
                (expression, expr_depth(k as usize, balanced))
            },
            |(expression, depth)| {
                let mesh = expression
                    .evaluate_with_context(STRICT)
                    .map(|o| o.into_value());
                Shaped::new((mesh, *depth), |(mesh, depth)| {
                    mesh.as_ref().ok().map(|mesh| Shape {
                        depth: *depth,
                        ..mesh_shape(mesh)
                    })
                })
            },
        );
    }

    atlas.operation(
        "solid boolean",
        "grid pattern",
        "instances",
        "Pattern a 16-segment cylinder on a square grid (disjoint copies merged into one mesh).",
    );
    let pin = solid::cylinder(ratio(1, 2), r(2), 16).expect("valid cylinder");
    atlas.sweep(
        "cylinder",
        &powers_of_two(0, 10),
        |n, _| {
            let side = n.sqrt().ceil() as usize;
            (side, (n as usize).div_ceil(side))
        },
        |&(rows, columns)| {
            shaped_mesh(solid::distribute_grid(&pin, rows, columns, r(2), r(2)))
        },
    );
}

fn manufacturing(atlas: &mut Atlas) {
    let sizes = vec![4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 64.0];
    atlas.operation(
        "manufacture",
        "slice to region",
        "sphere segments",
        "One horizontal slice of a sphere assembled into a filled region (the slicer's per-layer work).",
    );
    atlas.sweep(
        "sphere",
        &sizes,
        |s, _| sphere(s as usize),
        |mesh| shaped_region(solid::slice_z(mesh, ratio(1, 7)).map(|(region, _, _)| region)),
    );
    atlas.operation(
        "manufacture",
        "slice stack",
        "layers",
        "Slice a 16-segment sphere into evenly spaced layers of filled regions, as an additive or waterline slicer does; nodes are total boundary fragments.",
    );
    let body = sphere(16);
    atlas.sweep(
        "sphere",
        &powers_of_two(0, 8),
        |n, _| n as usize,
        |&layers| {
            let regions: Result<Vec<CurveRegion2>, _> = (0..layers)
                .map(|i| {
                    let z = ratio(2 * i as i64 + 1 - layers as i64, layers as i64);
                    solid::slice_z(&body, z).map(|(region, _, _)| region)
                })
                .collect();
            Shaped::new(regions, |regions| {
                regions.as_ref().ok().map(|regions| Shape {
                    nodes: regions.iter().map(|r| region_shape(r).nodes).sum(),
                    depth: regions.len() as u64,
                    bits: 0,
                })
            })
        },
    );
    atlas.operation(
        "manufacture",
        "flatten",
        "sphere segments",
        "Project every face of a sphere into its regularized XY union (a silhouette for nesting or fixturing).",
    );
    atlas.sweep(
        "sphere",
        &sizes,
        |s, _| sphere(s as usize),
        |mesh| shaped_region(solid::flatten(mesh)),
    );
    atlas.operation(
        "manufacture",
        "mass properties",
        "sphere segments",
        "Exact volume, centre of mass and inertia of a sphere at unit density; shape bits are the stored volume's.",
    );
    atlas.sweep(
        "sphere",
        &sizes,
        |s, _| sphere(s as usize),
        |mesh| {
            Shaped::new(solid::exact_mass_properties(mesh, 1), |report| {
                report.as_ref().ok().map(|report| {
                    let size = report.volume.representation_size(1 << 12);
                    Shape {
                        nodes: size.nodes as u64,
                        depth: size.depth as u64,
                        bits: size.max_rational_bits,
                    }
                })
            })
        },
    );
    atlas.operation(
        "manufacture",
        "rounded box",
        "ball segments",
        "Round a cube's edges by Minkowski sum with a ball of growing segment count.",
    );
    let block = solid::cube(r(2)).expect("valid cube");
    atlas.sweep(
        "cube ⊕ sphere",
        &[4.0, 6.0, 8.0, 12.0, 16.0],
        |s, _| solid::sphere(ratio(1, 4), s as usize, s as usize / 2).expect("valid ball"),
        |ball| shaped_mesh(solid::minkowski_sum(&block, ball)),
    );
    atlas.operation(
        "manufacture",
        "convex hull",
        "sphere segments",
        "Exact convex hull of a sphere's vertices.",
    );
    atlas.sweep(
        "sphere",
        &sizes,
        |s, _| sphere(s as usize),
        |mesh| shaped_mesh(solid::convex_hull(mesh)),
    );
    atlas.operation(
        "manufacture",
        "contains point",
        "sphere segments",
        "Exact point-in-solid test against a sphere.",
    );
    atlas.sweep(
        "sphere",
        &sizes,
        |s, _| sphere(s as usize),
        |mesh| solid::contains_point(mesh, &p3(0.1, 0.2, 0.3)),
    );

    let resolution = powers_of_two(3, 6);
    atlas.operation(
        "implicit",
        "sdf sphere",
        "samples per axis",
        "Sample an exact rational sphere field on an n^3 grid and mesh it with Surface Nets.",
    );
    atlas.sweep(
        "sphere field",
        &resolution,
        |n, _| n as usize,
        |&n| {
            shaped_mesh(solid::sdf(
                |p| &(&(&p.x * &p.x) + &(&p.y * &p.y)) + &(&(&p.z * &p.z) - &r(1)),
                (n, n, n),
                p3(-1.5, -1.5, -1.5),
                p3(1.5, 1.5, 1.5),
                0,
            ))
        },
    );
    atlas.operation(
        "implicit",
        "gyroid infill",
        "samples per axis",
        "Gyroid lattice solid inside a cube at growing sample resolution (lattice infill).",
    );
    let bounds = solid::cube(r(4)).expect("valid cube");
    atlas.sweep(
        "cube",
        &resolution,
        |n, _| n as usize,
        |&n| shaped_mesh(solid::gyroid_solid(&bounds, n, 1, 0, ratio(3, 10))),
    );
    atlas.operation(
        "implicit",
        "metaballs",
        "samples per axis",
        "Inverse-square metaballs (iso 1, padding 1/2) sampled exactly on an n^3 grid: rings of two and four balls whose centres are on a 1/8 grid, and two balls whose centres are binary64 cosines (long dyadics). Eight balls took 1.2 s (1/8 grid) and 6.6 s (binary64) on a 3^3 grid and are left out.",
    );
    for (kind, count, snapped) in [
        ("2 balls, 1/8-grid centres", 2usize, true),
        ("4 balls, 1/8-grid centres", 4, true),
        ("2 balls, binary64 centres", 2, false),
    ] {
        let balls: Vec<solid::MetaBall> = (0..count)
            .map(|i| {
                let theta = TAU * i as f64 / count as f64;
                let (x, y) = (theta.cos(), theta.sin());
                let centre = if snapped {
                    let snap = |v: f64| ratio((v * 8.0).round() as i64, 8);
                    Point3::new(snap(x), snap(y), r(0))
                } else {
                    p3(x, y, 0.0)
                };
                solid::MetaBall::new(centre, ratio(1, 2))
            })
            .collect();
        atlas.sweep(
            kind,
            &[3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0],
            |n, _| n as usize,
            |&n| shaped_mesh(solid::metaballs(&balls, (n, n, n), 1, ratio(1, 2))),
        );
    }

    atlas.operation(
        "manufacture",
        "hershey engraving",
        "characters",
        "Single-stroke Hershey text (Futura light) for engraving tool paths; nodes are strokes.",
    );
    let text = "The quick brown fox jumps over the lazy dog 0123456789 ";
    atlas.sweep(
        "futural",
        &powers_of_two(0, 10),
        |n, _| text.chars().cycle().take(n as usize).collect::<String>(),
        |line| {
            Shaped::new(
                curve::hershey_strings(line, &hershey::fonts::FUTURAL, 1),
                |strings| {
                    strings.as_ref().ok().map(|strings| Shape {
                        nodes: strings.len() as u64,
                        depth: 0,
                        bits: 0,
                    })
                },
            )
        },
    );
}

fn interchange(atlas: &mut Atlas) {
    let sizes = segments();
    atlas.operation(
        "export",
        "mesh export",
        "sphere segments",
        "Serialize a sphere (segments x segments/2) to each mesh format; shape nodes are output bytes.",
    );
    type Export = fn(&TriangleMesh) -> Result<usize, csgrs::io::IoError>;
    let exports: [(&str, Export); 7] = [
        ("STL binary", |m| {
            stl::to_stl_binary(m, "part").map(|b| b.len())
        }),
        ("STL ASCII", |m| stl::to_stl_ascii(m, "part").map(|s| s.len())),
        ("OBJ", |m| obj::to_obj(m, "part").map(|s| s.len())),
        ("PLY", |m| ply::to_ply(m, "part").map(|s| s.len())),
        ("AMF", |m| {
            amf::to_amf(m, "part", "millimeter").map(|s| s.len())
        }),
        ("glTF", |m| gltf::to_gltf(m, "part").map(|s| s.len())),
        ("DXF", |m| dxf::to_dxf(m).map(|b| b.len())),
    ];
    for (kind, export) in exports {
        atlas.sweep(
            kind,
            &sizes,
            |s, _| sphere(s as usize),
            move |mesh| shaped_bytes(export(mesh)),
        );
    }
    atlas.operation(
        "export",
        "export of irrational coordinates",
        "sphere segments",
        "Binary STL and OBJ export of a sphere rotated by 30 degrees, whose exact coordinates must be approximated.",
    );
    for (kind, export) in [exports[0], exports[2]] {
        atlas.sweep(
            kind,
            &sizes,
            |s, _| sphere(s as usize).rotated(0, 0, 30).expect("valid rotation"),
            move |mesh| shaped_bytes(export(mesh)),
        );
    }

    atlas.operation(
        "import",
        "mesh import",
        "sphere segments",
        "Parse a sphere exported to each format back into an exact mesh (with vertex welding and validation).",
    );
    atlas.sweep(
        "STL binary",
        &sizes,
        |s, _| stl::to_stl_binary(&sphere(s as usize), "part").expect("STL export"),
        |bytes| shaped_mesh(stl::from_stl(bytes)),
    );
    atlas.sweep(
        "OBJ",
        &sizes,
        |s, _| obj::to_obj(&sphere(s as usize), "part").expect("OBJ export"),
        |text| shaped_mesh(obj::from_obj(text.as_bytes())),
    );
    atlas.sweep(
        "glTF",
        &sizes,
        |s, _| gltf::to_gltf(&sphere(s as usize), "part").expect("glTF export"),
        |text| shaped_mesh(gltf::from_gltf(text.as_bytes()).map(|import| import.mesh)),
    );

    let teeth = powers_of_two(3, 12);
    let gear = |sides: f64| curve::regular_ngon(sides as usize, r(10)).expect("valid polygon");
    atlas.operation(
        "export",
        "2D export",
        "polygon sides",
        "Serialize a regular polygon (exact trigonometric vertices) to SVG and Gerber; shape nodes are output bytes. An 8-tooth involute gear took 162 ms to export and 3.3 s to re-import as SVG, so gears are left out.",
    );
    atlas.sweep(
        "SVG",
        &teeth,
        |t, _| gear(t),
        |region| shaped_bytes(svg::export_svg(region, &[], &[]).map(|s| s.len())),
    );
    atlas.sweep(
        "Gerber",
        &teeth,
        |t, _| gear(t),
        |region| shaped_bytes(gerber::export_gerber(region).map(|b| b.len())),
    );
    atlas.operation(
        "import",
        "2D import",
        "polygon sides",
        "Parse an exported regular polygon from SVG and Gerber back into a filled region.",
    );
    atlas.sweep(
        "SVG",
        &teeth,
        |t, _| svg::export_svg(&gear(t), &[], &[]).expect("SVG export"),
        |text| shaped_region(svg::import_svg(text).map(|(region, _, _)| region)),
    );
    atlas.sweep(
        "Gerber",
        &teeth,
        |t, _| gerber::export_gerber(&gear(t)).expect("Gerber export"),
        |bytes| shaped_region(gerber::import_gerber(bytes).map(|(region, _, _)| region)),
    );
}

fn adapter_union<A: ScalarAdapter>(
    segments: usize,
    radius: A::Scalar,
    offset: A::Scalar,
) -> (ScalarMesh<A>, ScalarMesh<A>)
where
    A::Scalar: Clone,
{
    let a =
        ScalarMesh::<A>::sphere(radius.clone(), segments, segments / 2).expect("valid sphere");
    let b = ScalarMesh::<A>::sphere(radius, segments, segments / 2)
        .and_then(|b| b.translate(offset.clone(), offset.clone(), offset))
        .expect("valid sphere");
    (a, b)
}

fn adapter_union_sweep<A: ScalarAdapter>(
    atlas: &mut Atlas,
    kind: &str,
    radius: A::Scalar,
    offset: A::Scalar,
) where
    A::Scalar: Clone,
{
    atlas.sweep(
        kind,
        &[4.0, 6.0, 8.0, 12.0, 16.0],
        |s, _| adapter_union::<A>(s as usize, radius.clone(), offset.clone()),
        |(a, b)| shaped_mesh(a.union(b).map(ScalarMesh::into_native)),
    );
}

fn adapter_buffers_sweep<A: ScalarAdapter>(atlas: &mut Atlas, kind: &str, radius: A::Scalar)
where
    A::Scalar: Clone,
{
    atlas.sweep(
        kind,
        &segments(),
        |s, _| {
            ScalarMesh::<A>::sphere(radius.clone(), s as usize, s as usize / 2)
                .expect("valid sphere")
        },
        |mesh| {
            mesh.vertices_and_indices()
                .map(|(vertices, _)| vertices.len())
        },
    );
}

fn adapters(atlas: &mut Atlas) {
    atlas.operation(
        "adapter",
        "adapter union",
        "sphere segments",
        "Union through the primitive-scalar facade over raw Real, f64, f32 and i128 scalars: operands are built from the backend's scalars and stay exact inside.",
    );
    adapter_union_sweep::<RawReal>(atlas, "raw Real", r(1), ratio(1, 3));
    adapter_union_sweep::<F64>(atlas, "f64", 1.0, 1.0 / 3.0);
    adapter_union_sweep::<F32>(atlas, "f32", 1.0, 1.0 / 3.0);
    adapter_union_sweep::<I128>(atlas, "i128", 4, 1);
    atlas.operation(
        "adapter",
        "adapter buffers",
        "sphere segments",
        "Convert an exact sphere (segments x segments/2, trigonometric coordinates) to each backend's scalar vertex and index buffers (i128 fails: the coordinates are not integral).",
    );
    adapter_buffers_sweep::<RawReal>(atlas, "raw Real", r(1));
    adapter_buffers_sweep::<F64>(atlas, "f64", 1.0);
    adapter_buffers_sweep::<F32>(atlas, "f32", 1.0);
    adapter_buffers_sweep::<I128>(atlas, "i128", 4);
}

fn caches(atlas: &mut Atlas) {
    atlas.operation(
        "cache",
        "sphere primitive cache",
        "sphere segments",
        "The same sphere primitive with grammar caching disabled (real work) and with the default policy (repeat calls are cache hits; residue shows retained entries).",
    );
    atlas.sweep(
        "cache disabled",
        &segments(),
        |s, _| s as usize,
        |&s| {
            cache::with_policy(CachePolicy::DISABLED, || {
                shaped_mesh(solid::sphere(r(1), s, s / 2))
            })
        },
    );
    atlas.sweep(
        "default cache",
        &segments(),
        |s, _| s as usize,
        |&s| shaped_mesh(solid::sphere(r(1), s, s / 2)),
    );
    atlas.operation(
        "cache",
        "slice cache retention",
        "sphere segments",
        "Slice a fresh sphere under the default policy: every call misses (new storage identity) but retains an entry; residue is the retained slice.",
    );
    atlas.sweep(
        "default cache",
        &segments(),
        |s, _| {
            let mesh = sphere(s as usize);
            TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec())
        },
        |mesh| shaped_region(solid::slice_z(mesh, ratio(1, 7)).map(|(region, _, _)| region)),
    );
}

fn main() {
    let mut atlas = Atlas::from_args("csgrs");
    cache::with_policy(CachePolicy::DISABLED, || {
        sketching(&mut atlas);
        construction(&mut atlas);
        solid_booleans(&mut atlas);
        manufacturing(&mut atlas);
        interchange(&mut atlas);
        adapters(&mut atlas);
    });
    // The cache probe runs under the default policy, which then persists
    // across calls (a per-call scope would evict entries on leaving it).
    caches(&mut atlas);
    atlas.finish();
}
